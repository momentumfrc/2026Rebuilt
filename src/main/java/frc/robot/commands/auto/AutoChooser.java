package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.MoPrefs;
import frc.robot.RobotPositioning;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.HoodSubsystem;
import frc.robot.subsystem.IndexerSubsystem;
import frc.robot.subsystem.KickerSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.NTHelpers;

public class AutoChooser {
    private enum AutoChoices {
        LEAVE,
        SHOOT
    }

    private enum ShootAutoRoutines {
        AUTO1
    } // edit with more choices once auto routines are defined

    private final RobotPositioning robotPositioning;
    private final DriveSubsystem driveSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final KickerSubsystem kickerSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final HoodSubsystem hoodSubsystem;

    private final BooleanEntry enableAutoSwitch;

    private SendableChooser<AutoChoices> autoChoicesChooser = NTHelpers.enumToChooser(AutoChoices.class);
    private SendableChooser<ShootAutoRoutines> autoRoutinesChooser = NTHelpers.enumToChooser(ShootAutoRoutines.class);

    public AutoChooser(
            RobotPositioning robotPositioning,
            DriveSubsystem driveSubsystem,
            TurretSubsystem turretSubsystem,
            IndexerSubsystem indexerSubsystem,
            KickerSubsystem kickerSubsystem,
            ShooterSubsystem shooterSubsystem,
            HoodSubsystem hoodSubsystem) {
        this.robotPositioning = robotPositioning;
        this.driveSubsystem = driveSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.kickerSubsystem = kickerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hoodSubsystem = hoodSubsystem;

        var autoTable = NTHelpers.getTable("Auto");
        enableAutoSwitch = NTHelpers.getBooleanEntry(autoTable, "Run Auto?", true);

        NTHelpers.publishSendable(autoTable, "Which Auto?", autoChoicesChooser);
        NTHelpers.publishSendable(autoTable, "Which Routine?", autoRoutinesChooser);
    }

    public Command buildLeaveAuto() {
        return Commands.run(() -> driveSubsystem.autoLeaveDrive(
                        new Translation2d(MoPrefs.autoLeaveSpeed.get().in(Units.Value), 0), 0))
                .withTimeout(null);
    }

    public Command buildAuto1() {
        return Commands.deadline(
                AutoPathPlannerCommands.getFollowPathCommand(driveSubsystem, robotPositioning, "path1"));
    } // fix this with actual path and subsystem commands once that is done

    public Command getAutoRoutine() {
        return switch (autoRoutinesChooser.getSelected()) {
            case AUTO1 -> buildAuto1();
        };
    }

    public Command getAutoChooserCommand() {
        if (!enableAutoSwitch.get()) {
            return Commands.print("Auto Disabled");
        }
        var auto =
                switch (autoChoicesChooser.getSelected()) {
                    case LEAVE -> buildLeaveAuto();
                    case SHOOT -> getAutoRoutine();
                };
        return auto;
    }
}
