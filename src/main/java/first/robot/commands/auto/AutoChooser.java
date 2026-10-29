package first.robot.commands.auto;

import com.pathplanner.lib.path.PathConstraints;
import first.robot.RobotPositioning;
import first.robot.molib.NTHelpers;
import first.robot.subsystem.DriveSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.tunable.Selectable;
import org.wpilib.units.Units;

public class AutoChooser {
    private static final PathConstraints AUTO_PATH_CONSTRAINTS = new PathConstraints(
            Units.MetersPerSecond.of(2),
            Units.MetersPerSecondPerSecond.of(2),
            Units.RotationsPerSecond.of(2),
            Units.RotationsPerSecondPerSecond.of(2));

    private enum ShootAutoRoutines {
        TEST_PATH
    } // edit with more choices once auto routines are defined

    private final RobotPositioning robotPositioning;
    private final DriveSubsystem driveSubsystem;

    private final BooleanEntry enableAutoSwitch;

    private Selectable<ShootAutoRoutines> autoRoutinesChooser = NTHelpers.enumToChooser(ShootAutoRoutines.class);
    private BooleanEntry assumeRobotPose;

    public AutoChooser(RobotPositioning robotPositioning, DriveSubsystem driveSubsystem) {
        this.robotPositioning = robotPositioning;
        this.driveSubsystem = driveSubsystem;

        var autoTable = NTHelpers.getTable("Auto");
        enableAutoSwitch = NTHelpers.getBooleanEntry(autoTable, "Run Auto?", true);

        NTHelpers.publishSendable(autoTable, "Which Routine?", autoRoutinesChooser);
        assumeRobotPose = NTHelpers.getBooleanEntry(autoTable, "Assume Robot Position?", false);
    }

    public Command getAutoRoutine() {
        return switch (autoRoutinesChooser.getSelected()) {
            case TEST_PATH ->
                AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "TEST", assumeRobotPose.get());
        };
    }

    public Command getAutoChooserCommand() {
        if (!enableAutoSwitch.get()) {
            return Commands.print("Auto Disabled");
        }
        return getAutoRoutine();
    }
}
