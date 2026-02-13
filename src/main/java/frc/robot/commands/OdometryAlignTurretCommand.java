package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.OdometryTargetingHelper;

public class OdometryAlignTurretCommand extends Command {
    private final TurretSubsystem turret;
    private final PositioningSubsystem positioning;

    private final MutAngle setpointAngle = Units.Radians.mutable(0);

    public OdometryAlignTurretCommand(TurretSubsystem turret, PositioningSubsystem positioning) {
        this.turret = turret;
        this.positioning = positioning;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose2d robotPose = positioning.getRobotPose();
        Translation2d toTarget = OdometryTargetingHelper.getTranslationToTarget(
                robotPose.getTranslation(), DriverStation.getAlliance().orElse(DriverStation.Alliance.Red));
        setpointAngle.mut_replace(toTarget.getAngle().getRadians(), Units.Radians);
        setpointAngle.mut_minus(robotPose.getRotation().getRadians(), Units.Radians);
        turret.alignAbsolute(setpointAngle);
    }

    @Override
    public boolean isFinished() {
        // TODO
        return false;
    }
}
