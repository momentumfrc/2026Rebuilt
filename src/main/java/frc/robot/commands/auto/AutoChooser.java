package frc.robot.commands.auto;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.MoPrefs;
import frc.robot.RobotPositioning;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.intake.RollerCommands;
import frc.robot.commands.intake.WristCommands;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.HoodSubsystem;
import frc.robot.subsystem.IndexerSubsystem;
import frc.robot.subsystem.IntakeRollerSubsystem;
import frc.robot.subsystem.IntakeWristSubsystem;
import frc.robot.subsystem.KickerSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.NTHelpers;

public class AutoChooser {
    private enum AutoChoices {
        SHOOT
    } //probably won't need this, leaving it here just in case we do for whatever reason

    private enum ShootAutoRoutines {
        CENTER_AND_SCORE,
        SCORE_LEFT_TO_HUB,
        SCORE_RIGHT_TO_HUB,
        SCORE_LEFT_AND_COLLECT_DEPOT,
        SCORE_LEFT_AND_COLLECT_NEUTRAL_ZONE,
        SCORE_RIGHT_AND_COLLECT_NEUTRAL_ZONE
    } // edit with more choices once auto routines are defined

    private final RobotPositioning robotPositioning;
    private final DriveSubsystem driveSubsystem;
    private final TurretSubsystem turretSubsystem;
    private final IndexerSubsystem indexerSubsystem;
    private final KickerSubsystem kickerSubsystem;
    private final ShooterSubsystem shooterSubsystem;
    private final HoodSubsystem hoodSubsystem;
    private final IntakeRollerSubsystem intakeRollerSubsystem;
    private final IntakeWristSubsystem intakeWristSubsystem;

    private final ShootCommand shootCommand;

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
            HoodSubsystem hoodSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            IntakeWristSubsystem intakeWristSubsystem,
            ShootCommand shootCommand) {
        this.robotPositioning = robotPositioning;
        this.driveSubsystem = driveSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.kickerSubsystem = kickerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.intakeWristSubsystem = intakeWristSubsystem;

        this.shootCommand = shootCommand;

        var autoTable = NTHelpers.getTable("Auto");
        enableAutoSwitch = NTHelpers.getBooleanEntry(autoTable, "Run Auto?", true);

        NTHelpers.publishSendable(autoTable, "Which Auto?", autoChoicesChooser);
        NTHelpers.publishSendable(autoTable, "Which Routine?", autoRoutinesChooser);
    }

    public Command getShootCommand() {
        return Commands.deadline(
                shootCommand.withTimeout(MoPrefs.autoShooterRunTime.get().in(Units.Seconds)),
                indexerSubsystem.run(indexerSubsystem::run));
    }

    public Command getIntakeCommand() {
        return WristCommands.deployIntakeWristCommand(intakeWristSubsystem)
                .andThen(RollerCommands.runIntakeRollerCommand(intakeRollerSubsystem)
                        .withTimeout(MoPrefs.autoIntakeRunTime.get().in(Units.Seconds)));
    }

    // We shoot the fuel, and then we move out of the way in case another alliance member needs to move there (unlikely but just incase)
    public Command buildCenterAuto() {
        return getShootCommand()
                .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Center Forward"));
    }

    // Might need some fixing later
    public Command buildScoreLeftAuto() {
        return Commands.deadline(
                AutoPathPlannerCommands.getFollowPathCommand(driveSubsystem, robotPositioning, "Left Start to Left Hub")
                        .andThen(getShootCommand()));
    }

    public Command buildScoreRightAuto() {
        return Commands.deadline(AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Right Start to Right Hub")
                .andThen(getShootCommand()));
    }

    // might be redundant, there is probably a better way to write this
    public Command buildScoreLeftAndDepot() {
        return Commands.deadline(
                AutoPathPlannerCommands.getFollowPathCommand(driveSubsystem, robotPositioning, "Left Start to Left Hub")
                        .andThen(getShootCommand())
                        .andThen(Commands.parallel(
                                        AutoPathPlannerCommands.getFollowPathCommand(
                                                driveSubsystem, robotPositioning, "Left Hub to Depot"),
                                        getIntakeCommand())
                                .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                                        driveSubsystem, robotPositioning, "Depot to Left Hub"))
                                .andThen(getShootCommand())));
    }

    public Command buildScoreLeftAndNeutral() {
        return Commands.deadline(
                AutoPathPlannerCommands.getFollowPathCommand(driveSubsystem, robotPositioning, "Left Start to Left Hub")
                        .andThen(getShootCommand())
                        .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                                driveSubsystem, robotPositioning, "Left Hub to Left Neutral Zone"))
                        .andThen(Commands.parallel(
                                AutoPathPlannerCommands.getFollowPathCommand(
                                        driveSubsystem, robotPositioning, "Left Neutral Zone to Left Hub"),
                                getIntakeCommand()))
                        .andThen(getShootCommand()));
    }

    public Command buildScoreRightAndNeutral() {
        return Commands.deadline(AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Right Start to Right Hub")
                .andThen(getShootCommand())
                .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Right Hub to Right Neutral Zone"))
                .andThen(Commands.parallel(
                        AutoPathPlannerCommands.getFollowPathCommand(
                                driveSubsystem, robotPositioning, "Right Neutral Zone to Right Hub"),
                        getIntakeCommand()))
                .andThen(getShootCommand()));
    }

    public Command getAutoRoutine() {
        return switch (autoRoutinesChooser.getSelected()) {
            case CENTER_AND_SCORE -> buildCenterAuto();
            case SCORE_LEFT_TO_HUB -> buildScoreLeftAuto();
            case SCORE_RIGHT_TO_HUB -> buildScoreRightAuto();
            case SCORE_LEFT_AND_COLLECT_DEPOT -> buildScoreLeftAndDepot();
            case SCORE_LEFT_AND_COLLECT_NEUTRAL_ZONE -> buildScoreLeftAndNeutral();
            case SCORE_RIGHT_AND_COLLECT_NEUTRAL_ZONE -> buildScoreRightAndNeutral();
        };
    }

    public Command getAutoChooserCommand() {
        if (!enableAutoSwitch.get()) {
            return Commands.print("Auto Disabled");
        }
        var auto =
                switch (autoChoicesChooser.getSelected()) {
                    case SHOOT -> getAutoRoutine();
                };
        return auto;
    }
}
