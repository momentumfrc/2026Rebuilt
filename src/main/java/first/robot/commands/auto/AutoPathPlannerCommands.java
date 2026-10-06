package first.robot.commands.auto;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import first.robot.RobotPositioning;
import first.robot.subsystem.DriveSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

public class AutoPathPlannerCommands {

    public static RobotConfig pathPlannerConfig;

    public static Command getFollowPathCommand(
            DriveSubsystem driveSubsystem, RobotPositioning robotPositioning, PathPlannerPath path) {
        PathFollowingController driveController = driveSubsystem.driveController();

        try {
            pathPlannerConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            DriverStationErrors.reportError("Failed to Load PathPlanner Config - ", e.getStackTrace());
            return Commands.none();
        }

        Command pathFollowingCommand = new FollowPathCommand(
                path,
                robotPositioning::getRobotPose,
                robotPositioning::getRobotVelocity,
                driveSubsystem::driveRobotRelativeSpeeds,
                driveController,
                pathPlannerConfig,
                () -> MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED,
                driveSubsystem);

        pathFollowingCommand = Commands.either(
                pathFollowingCommand,
                Commands.print("Refusing to follow path without established initial position"),
                robotPositioning::hasInitialPosition);

        return pathFollowingCommand.asProxy();
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
                        Commands.runOnce(() -> {
                            Pose2d pose;
                            if (MatchState.getAlliance().orElse(Alliance.RED) == Alliance.RED) {
                                pose = FlippingUtil.flipFieldPose(startPose);
                            } else {
                                pose = startPose;
                            }
                            robotPositioning.resetOdometry(pose);
                        }),
                        getFollowPathCommand(driveSubsystem, robotPositioning, path));
            } else {
                return getFollowPathCommand(driveSubsystem, robotPositioning, path);
            }

        } catch (Exception e) {
            DriverStationErrors.reportError(
                    "Failed to build and follow PathPlanner command - " + e.getMessage(), e.getStackTrace());
            return Commands.print("Failed to load pathplanner config, refusing to follow provided path");
        }
    }
}
