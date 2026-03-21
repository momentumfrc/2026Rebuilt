package frc.robot.commands;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MoPrefs;
import frc.robot.molib.NTHelpers;
import frc.robot.shootutils.TurretTargeting;
import frc.robot.subsystem.HoodSubsystem;
import frc.robot.subsystem.IndexerSubsystem;
import frc.robot.subsystem.KickerSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.OdometryTargetingHelper;

public class ShootCommand extends Command {
    private final KickerSubsystem kicker;
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;

    private final TurretTargeting targeting;

    private final Alert targetOutOfRange = new Alert("Target out of turret range", Alert.AlertType.kInfo);

    private enum TargetingMode {
        ON_THE_MOVE,
        STATIONARY,
        FALLBACK
    };

    private final SendableChooser<TargetingMode> modeChooser =
            NTHelpers.enumToChooser(TargetingMode.class, TargetingMode.ON_THE_MOVE);

    private final BooleanEntry doOverrideFlywheelSetpoint;
    private final DoubleEntry overrideFlywheelSetpoint;
    private final BooleanEntry doOverrideHoodSetpoint;
    private final DoubleEntry overrideHoodSetpoint;
    private final BooleanEntry currentlyShooting;

    private final MutAngle hoodOverridePosition = Units.Degrees.mutable(0);
    private final MutAngularVelocity flywheelOverrideSpeed = Units.RPM.mutable(0);

    public ShootCommand(
            TurretTargeting targeting,
            IndexerSubsystem indexer,
            KickerSubsystem kicker,
            TurretSubsystem turret,
            ShooterSubsystem shooter,
            HoodSubsystem hood) {
        this.targeting = targeting;
        this.kicker = kicker;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        var turretTable = NTHelpers.getTable("turret");
        NTHelpers.publishSendable(turretTable, "Targeting Mode", modeChooser);

        var shooterFlywheelTable = NTHelpers.getTable("shooter-flywheel");
        doOverrideFlywheelSetpoint =
                NTHelpers.getBooleanEntry(shooterFlywheelTable, "Override Flywheel Setpoint?", false);
        overrideFlywheelSetpoint =
                NTHelpers.getDoubleEntry(shooterFlywheelTable, "Flywheel Override Setpoint (RPM)", 120);

        currentlyShooting =
                shooterFlywheelTable.getBooleanTopic("Currently Shooting?").getEntry(false);

        var shooterHoodTable = NTHelpers.getTable("shooter-hood");
        doOverrideHoodSetpoint = NTHelpers.getBooleanEntry(shooterHoodTable, "Override Hood Setpoint?", false);
        overrideHoodSetpoint = NTHelpers.getDoubleEntry(shooterHoodTable, "Hood Override Setpoint (°)", 10);

        addRequirements(kicker, turret, shooter, hood);
    }

    private Angle getHoodOverridePosition() {
        return hoodOverridePosition.mut_replace(overrideHoodSetpoint.get(), Units.Degrees);
    }

    private AngularVelocity getFlywheelOverrideSpeed() {
        return flywheelOverrideSpeed.mut_replace(overrideFlywheelSetpoint.get(), Units.RPM);
    }

    public boolean readyToShoot() {
        return turret.targetIsAligned() && hood.isInPosition() && shooter.isUpToSpeed();
    }

    public boolean currentlyShooting() {
        return currentlyShooting.get();
    }

    @Override
    public void execute() {
        var targetingMode = modeChooser.getSelected();
        if (targetingMode == TargetingMode.FALLBACK) {
            var targetAngle = MoPrefs.turretFallbackSetpoint.get();
            var moduloAngle = turret.getAngleHelper().turretAngleModulus(targetAngle);
            if (moduloAngle.inRange() == false) {
                targetOutOfRange.set(true);
                currentlyShooting.set(false);

                kicker.stop();
                shooter.stop();
                hood.goToRest();
                turret.stop();
                return;
            }

            turret.alignAbsolute(targetAngle, Units.DegreesPerSecond.zero());
            hood.setPosition(MoPrefs.hoodFallbackSetpoint.get());
            shooter.runAtSpeed(MoPrefs.flywheelFallbackSetpoint.get());

        } else {
            var target = OdometryTargetingHelper.getTarget(
                    DriverStation.getAlliance().orElse(Alliance.Red));

            var firingSolution =
                    switch (targetingMode) {
                        case ON_THE_MOVE -> targeting.targetPositionSOTM(target.toTranslation2d());
                        case STATIONARY -> targeting.targetPositionStationary(target.toTranslation2d());
                        default -> throw new IllegalArgumentException("Unexpected value: " + targetingMode);
                    };
            var moduloAngle = turret.getAngleHelper().turretAngleModulus(firingSolution.goalAngle());
            if (moduloAngle.inRange() == false) {
                targetOutOfRange.set(true);
                currentlyShooting.set(false);

                kicker.stop();
                shooter.stop();
                hood.goToRest();
                turret.align(firingSolution);
                return;
            }

            turret.align(firingSolution);
            if (doOverrideHoodSetpoint.get()) {
                hood.setPosition(getHoodOverridePosition());
            } else {
                hood.setCalculatedPosition(firingSolution.targetDistance());
            }
            if (doOverrideFlywheelSetpoint.get()) {
                shooter.runAtSpeed(getFlywheelOverrideSpeed());
            } else {
                shooter.runAtCalculatedSpeed(firingSolution.targetDistance());
            }
        }

        targetOutOfRange.set(false);

        if (readyToShoot()) {
            kicker.run();

            currentlyShooting.set(true);
        } else {
            kicker.stop();

            currentlyShooting.set(false);
        }
    }

    public void end() {
        targetOutOfRange.set(false);
    }
}
