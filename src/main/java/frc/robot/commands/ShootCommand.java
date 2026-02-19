package frc.robot.commands;

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
import frc.robot.util.TurretAngleHelper;

public class ShootCommand extends Command {
    private enum TurretAlignMode {
        ABSOLUTE,
        RELATIVE
    }

    private final SendableChooser<TurretAlignMode> alignModeChooser = NTHelpers.enumToChooser(TurretAlignMode.class);
    private final SendableChooser<TurretTargeting.TurretTargetMode> targetModeChooser =
            NTHelpers.enumToChooser(TurretTargeting.TurretTargetMode.class);

    private final KickerSubsystem kicker;
    private final TurretSubsystem turret;
    private final ShooterSubsystem shooter;
    private final HoodSubsystem hood;

    private final TurretTargeting targeting;
    private final TurretAngleHelper angleHelper;

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

        this.angleHelper = turret.getAngleHelper();

        addRequirements(kicker, turret, shooter, hood);
    }

    private void alignAbsolute(TurretTargeting.TurretSetpoint firingSolution) {
        turret.alignAbsolute(firingSolution);
        hood.setCalculatedPosition(firingSolution.targetDistance());
        shooter.runAtSpeed(firingSolution.goalVelocity());

        if (turret.absoluteTargetIsAligned() && hood.isInPosition() && shooter.isUpToSpeed()) {
            kicker.run();
        } else {
            kicker.stop();
        }
    }

    private void alignRelative(TurretTargeting.TurretSetpoint firingSolution) {
        if (turret.relativeTargetIsVisible()) {
            turret.alignRelative();
        } else {
            turret.alignAbsolute(firingSolution);
        }

        hood.setCalculatedPosition(firingSolution.targetDistance());
        shooter.runAtSpeed(firingSolution.goalVelocity());

        if (turret.relativeTargetIsAligned() && hood.isInPosition() && shooter.isUpToSpeed()) {
            kicker.run();
        } else {
            kicker.stop();
        }
    }

    @Override
    public void execute() {
        var target =
                OdometryTargetingHelper.getTarget(DriverStation.getAlliance().orElse(Alliance.Red));
        var firingSolution = targeting.targetPosition(target.toTranslation2d(), targetModeChooser.getSelected());
        var moduloAngle = angleHelper.turretAngleModulus(firingSolution.goalAngle());
        if (moduloAngle == null) {
            // Solution is out of the turret's range
            kicker.stop();
            turret.stop();
            shooter.stop();
            hood.setPosition(MoPrefs.hoodDeadzonePosition.get());
            return;
        }

        switch (alignModeChooser.getSelected()) {
            case ABSOLUTE -> alignAbsolute(firingSolution);
            case RELATIVE -> alignRelative(firingSolution);
        }
    }
}
