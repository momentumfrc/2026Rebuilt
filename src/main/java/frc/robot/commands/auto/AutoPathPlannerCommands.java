package frc.robot.commands.auto;

import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotPositioning;
import frc.robot.subsystem.DriveSubsystem;

public class AutoPathPlannerCommands {

    public AutoPathPlannerCommands(RobotPositioning robotPositioning, DriveSubsystem driveSubsystem) {

        // auto-builder config based on PathPlannerLIB, we can change to another auto follower if we need to
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    robotPositioning::getRobotPose,
                    robotPositioning::resetOdometry,
                    robotPositioning::getRobotVelocity,
                    (speeds, feedforwards) -> driveSubsystem.driveRobotRelativeSpeeds(speeds, feedforwards),
                    new PPHolonomicDriveController(
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                            ),
                    config, // The robot configuration
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }
                        return false;
                    },
                    driveSubsystem // Reference to this subsystem to set requirements
                    );
        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to build and follow PathPlanner command - " + e.getMessage(), e.getStackTrace());
        }
    } // TODO: AUTO PIDS

    public static Command getFollowPathCommand(
            DriveSubsystem driveSubsystem,
            RobotPositioning robotPositioning,
            String pathName,
            boolean assumeRobotPosition) {
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            if (assumeRobotPosition) {
                Pose2d startPose = path.getStartingHolonomicPose().orElseGet(() -> new Pose2d(path.getPoint(0).position, Rotation2d.kZero));
                return new SequentialCommandGroup(Commands.runOnce(() -> robotPositioning.resetOdometry(startPose)), AutoBuilder.followPath(path));
            } else {
            return AutoBuilder.followPath(path);
        }

        } catch (Exception e) {
            DriverStation.reportError(
                    "Failed to build and follow PathPlanner command - " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}
