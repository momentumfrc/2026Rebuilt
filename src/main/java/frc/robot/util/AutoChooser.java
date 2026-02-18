package frc.robot.util;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotPositioning;
import frc.robot.subsystem.DriveSubsystem;

public class AutoChooser {
    private final RobotPositioning robotPositioning;
    private final DriveSubsystem driveSubsystem;

    private final BooleanEntry enableAutoSwitch;

    public AutoChooser(RobotPositioning robotPositioning, DriveSubsystem driveSubsystem) {
        this.robotPositioning = robotPositioning;
        this.driveSubsystem = driveSubsystem;

        var autoTable = NTHelpers.getTable("Auto");
        enableAutoSwitch = NTHelpers.getBooleanEntry(autoTable, "Run Auto?", true);
    }

    public Command getAutoChooserCommand() {
        if (!enableAutoSwitch.get()) {
            return Commands.print("Auto Disabled");
        }
        return null;
    }
}
