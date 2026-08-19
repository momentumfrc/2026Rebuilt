package first.commands;

import first.MoPrefs;
import first.molib.NTHelpers;
import first.shootutils.TurretTargeting;
import first.subsystem.HoodSubsystem;
import first.subsystem.KickerSubsystem;
import first.subsystem.ShooterSubsystem;
import first.subsystem.TurretSubsystem;
import first.util.OdometryTargetingHelper;
import org.wpilib.command2.Command;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.driverstation.Alert;
import org.wpilib.driverstation.Alliance;

public class ShootCommand extends Command {
    private final KickerSubsystem kicker;
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;

    private final TurretTargeting targeting;

    private final OdometryTargetingHelper.TargetType targetType;

    private final Alert targetOutOfRange = new Alert("Target out of turret range", Alert.Level.LOW);

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

    private final Angle hoodOverridePosition = Units.Degrees.of(0);
    private final AngularVelocity flywheelOverrideSpeed = Units.RPM.of(0);

    public ShootCommand(
            OdometryTargetingHelper.TargetType targetType,
            TurretTargeting targeting,
            KickerSubsystem kicker,
            TurretSubsystem turret,
            ShooterSubsystem shooter,
            HoodSubsystem hood) {
        this.targetType = targetType;
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

        var shooterHoodTable = NTHelpers.getTable("shooter-hood");
        doOverrideHoodSetpoint = NTHelpers.getBooleanEntry(shooterHoodTable, "Override Hood Setpoint?", false);
        overrideHoodSetpoint = NTHelpers.getDoubleEntry(shooterHoodTable, "Hood Override Setpoint (°)", 10);

        addRequirements(kicker, turret, shooter, hood);
    }

    public static ShootCommand getHubShootCommand(
            TurretTargeting targeting,
            KickerSubsystem kicker,
            TurretSubsystem turret,
            ShooterSubsystem shooter,
            HoodSubsystem hood) {
        return new ShootCommand(OdometryTargetingHelper.TargetType.HUB, targeting, kicker, turret, shooter, hood);
    }

    public static ShootCommand getShuttleShootCommand(
            TurretTargeting targeting,
            KickerSubsystem kicker,
            TurretSubsystem turret,
            ShooterSubsystem shooter,
            HoodSubsystem hood) {
        return new ShootCommand(OdometryTargetingHelper.TargetType.SHUTTLE, targeting, kicker, turret, shooter, hood);
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

    @Override
    public void execute() {
        var targetingMode = modeChooser.getSelected();
        if (targetingMode == TargetingMode.FALLBACK) {
            var targetAngle = MoPrefs.turretFallbackSetpoint.get();
            var moduloAngle = turret.getAngleHelper().turretAngleModulus(targetAngle);
            if (moduloAngle.inRange() == false) {
                targetOutOfRange.set(true);

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
                    MatchState.getAlliance().orElse(Alliance.RED),
                    targeting.getPositioning().getRobotPose(),
                    targetType);

            var firingSolution =
                    switch (targetingMode) {
                        case ON_THE_MOVE -> targeting.targetPositionSOTM(target.toTranslation2d());
                        case STATIONARY -> targeting.targetPositionStationary(target.toTranslation2d());
                        default -> throw new IllegalArgumentException("Unexpected value: " + targetingMode);
                    };
            var moduloAngle = turret.getAngleHelper().turretAngleModulus(firingSolution.goalAngle());
            if (moduloAngle.inRange() == false) {
                targetOutOfRange.set(true);

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
        } else {
            kicker.stop();
        }
    }

    public void end() {
        targetOutOfRange.set(false);
    }
}
