package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MoPrefs;
import frc.robot.shootutils.TurretTargeting;
import frc.robot.subsystem.HoodSubsystem;
import frc.robot.subsystem.KickerSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.NTHelpers;
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

    public ShootCommand(
            TurretTargeting targeting,
            KickerSubsystem kicker,
            TurretSubsystem turret,
            ShooterSubsystem shooter,
            HoodSubsystem hood) {
        this.targeting = targeting;
        this.kicker = kicker;
        this.turret = turret;
        this.shooter = shooter;
        this.hood = hood;

        var table = NTHelpers.getTable("turret");
        NTHelpers.publishSendable(table, "Targeting Mode", modeChooser);

        addRequirements(kicker, turret, shooter, hood);
    }

    private void outOfRange() {
        kicker.stop();
        turret.stop();
        shooter.stop();
        hood.setPosition(MoPrefs.hoodDeadzonePosition.get());
        targetOutOfRange.set(true);
    }

    @Override
    public void execute() {
        var targetingMode = modeChooser.getSelected();
        if (targetingMode == TargetingMode.FALLBACK) {
            var targetAngle = MoPrefs.turretFallbackSetpoint.get();
            var moduloAngle = turret.getAngleHelper().turretAngleModulus(targetAngle);
            if (moduloAngle == null) {
                outOfRange();
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
            if (moduloAngle == null) {
                outOfRange();
                return;
            }

            turret.align(firingSolution);
            hood.setCalculatedPosition(firingSolution.targetDistance());
            shooter.runAtCalculatedSpeed(firingSolution.targetDistance());
        }

        targetOutOfRange.set(false);

        if (turret.targetIsAligned() && hood.isInPosition() && shooter.isUpToSpeed()) {
            kicker.run();
        } else {
            kicker.stop();
        }
    }

    public void end() {
        targetOutOfRange.set(false);
    }
}
