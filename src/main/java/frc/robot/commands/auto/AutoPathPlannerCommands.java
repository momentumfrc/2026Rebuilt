package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
            e.printStackTrace();
        }
    } // TODO: AUTO PIDS

    public static Command getFollowPathCommand(
            DriveSubsystem driveSubsystem, RobotPositioning robotPositioning, String pathName) {
        try {
            // Load the path you want to follow using its name in the GUI
            PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

            return AutoBuilder.followPath(path);
        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }
}
