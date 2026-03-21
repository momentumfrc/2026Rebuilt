package frc.robot.commands.auto;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPositioning;
import frc.robot.subsystem.DriveSubsystem;

public class AutoPathPlannerCommands {

    public static RobotConfig pathPlannerConfig;

    public static Command getFollowPathCommand(
            DriveSubsystem driveSubsystem, RobotPositioning robotPositioning, PathPlannerPath path) {
        PathFollowingController driveController = driveSubsystem.driveController();

        try {
            pathPlannerConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStation.reportError("Failed to Load PathPlanner Config - ", e.getStackTrace());
        }

        FollowPathCommand pathFollowingCommand = new FollowPathCommand(
                path,
                robotPositioning::getRobotPose,
                robotPositioning::getRobotVelocity,
                driveSubsystem::driveRobotRelativeSpeeds,
                driveController,
                pathPlannerConfig,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                driveSubsystem);

        return pathFollowingCommand;
    }

    public static Command getFollowPathCommand(
            DriveSubsystem driveSubsystem,
            RobotPositioning robotPositioning,
            String pathName,
            boolean assumeRobotPosition) {

        try {
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            if (assumeRobotPosition) {
                Pose2d startPose = path.getStartingHolonomicPose()
                        .orElseGet(() -> new Pose2d(path.getPoint(0).position, Rotation2d.kZero));
                return new SequentialCommandGroup(
                        Commands.runOnce(() -> robotPositioning.resetOdometry(startPose)),
                        getFollowPathCommand(driveSubsystem, robotPositioning, path));
            } else {
                return getFollowPathCommand(driveSubsystem, robotPositioning, path);
            }

        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to build and follow PathPlanner command - " + e.getMessage(), e.getStackTrace());
            return Commands.print("Failed to load pathplanner config, refusing to follow provided path");
        }
    }
}
