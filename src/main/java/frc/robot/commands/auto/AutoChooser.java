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
import frc.robot.molib.NTHelpers;
import frc.robot.shootutils.TurretTargeting;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.HoodSubsystem;
import frc.robot.subsystem.IndexerSubsystem;
import frc.robot.subsystem.IntakeRollerSubsystem;
import frc.robot.subsystem.IntakeWristSubsystem;
import frc.robot.subsystem.KickerSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import java.util.Collections;

public class AutoChooser {
    private enum AutoChoices {
        SHOOT
    } // probably won't need this, leaving it here just in case we do for whatever reason

    private enum ShootAutoRoutines {
        CENTER_AND_SCORE,
        SCORE_LEFT_TO_HUB,
        SCORE_RIGHT_TO_HUB,
        SCORE_LEFT_AND_COLLECT_DEPOT,
        SCORE_LEFT_AND_COLLECT_NEUTRAL_ZONE,
        SCORE_RIGHT_AND_COLLECT_NEUTRAL_ZONE,
        SCORE_RIGHT_AND_COLLECT_OUTPOST
    } // edit with more choices once auto routines are defined

    private final TurretTargeting turretTargeting;

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
    private BooleanEntry assumeRobotPose;

    public AutoChooser(
            RobotPositioning robotPositioning,
            DriveSubsystem driveSubsystem,
            TurretSubsystem turretSubsystem,
            IndexerSubsystem indexerSubsystem,
            KickerSubsystem kickerSubsystem,
            ShooterSubsystem shooterSubsystem,
            HoodSubsystem hoodSubsystem,
            IntakeRollerSubsystem intakeRollerSubsystem,
            IntakeWristSubsystem intakeWristSubsystem) {
        this.robotPositioning = robotPositioning;
        this.driveSubsystem = driveSubsystem;
        this.turretSubsystem = turretSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        this.kickerSubsystem = kickerSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.intakeRollerSubsystem = intakeRollerSubsystem;
        this.intakeWristSubsystem = intakeWristSubsystem;

        turretTargeting = new TurretTargeting(robotPositioning);
        shootCommand = new ShootCommand(
                turretTargeting, indexerSubsystem, kickerSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem);

        var autoTable = NTHelpers.getTable("Auto");
        enableAutoSwitch = NTHelpers.getBooleanEntry(autoTable, "Run Auto?", true);

        NTHelpers.publishSendable(autoTable, "Which Auto?", autoChoicesChooser);
        NTHelpers.publishSendable(autoTable, "Which Routine?", autoRoutinesChooser);
        assumeRobotPose = NTHelpers.getBooleanEntry(autoTable, "Assume Robot Position?", false);
    }

    public Command getShootCommand() {
        return shootCommand.alongWith(
                Commands.defer(() -> Commands.waitUntil(shootCommand::readyToShoot), Collections.emptySet())
                        .andThen(indexerSubsystem.run(indexerSubsystem::run)));
    }

    public Command getIntakeCommand() {
        return WristCommands.deployIntakeWristCommand(intakeWristSubsystem)
                .andThen(RollerCommands.runIntakeRollerCommand(intakeRollerSubsystem));
    }

    // We shoot the fuel, and then we move out of the way in case another alliance member needs to move there (unlikely
    // but just incase)
    public Command buildCenterAuto() {
        return getShootCommand()
                .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Center Forward", assumeRobotPose.get()));
    }

    // Might need some fixing later
    public Command buildScoreLeftAuto() {
        return AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Left Start to Left Hub", assumeRobotPose.get())
                .andThen(getShootCommand());
    }

    public Command buildScoreRightAuto() {
        return AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Right Start to Right Hub", assumeRobotPose.get())
                .andThen(getShootCommand());
    }

    // might be redundant, there is probably a better way to write this
    public Command buildScoreLeftAndDepot() {
        return AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Left Start to Left Hub", assumeRobotPose.get())
                .andThen(getShootCommand())
                .andThen(Commands.deadline(
                                AutoPathPlannerCommands.getFollowPathCommand(
                                        driveSubsystem, robotPositioning, "Left Hub to Depot", false),
                                getIntakeCommand())
                        .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                                driveSubsystem, robotPositioning, "Depot to Left Hub", false))
                        .andThen(getShootCommand()));
    }

    public Command buildScoreLeftAndNeutral() {
        return AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Left Start to Left Hub", assumeRobotPose.get())
                .andThen(getShootCommand())
                .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Left Hub to Left Neutral Zone", false))
                .andThen(Commands.deadline(
                        AutoPathPlannerCommands.getFollowPathCommand(
                                driveSubsystem, robotPositioning, "Left Neutral Zone to Left Hub", false),
                        getIntakeCommand()))
                .andThen(getShootCommand());
    }

    public Command buildScoreRightAndNeutral() {
        return AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Right Start to Right Hub", assumeRobotPose.get())
                .andThen(getShootCommand())
                .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Right Hub to Right Neutral Zone", false))
                .andThen(Commands.deadline(
                        AutoPathPlannerCommands.getFollowPathCommand(
                                driveSubsystem, robotPositioning, "Right Neutral Zone to Right Hub", false),
                        getIntakeCommand()))
                .andThen(getShootCommand());
    }

    public Command buildScoreRightAndOutpost() {
        return AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Right Hub to Outpost", assumeRobotPose.get())
                .andThen(shootCommand.andThen(AutoPathPlannerCommands.getFollowPathCommand(
                                driveSubsystem, robotPositioning, "Outpost to Right Hub", false)
                        .andThen(Commands.waitSeconds(
                                MoPrefs.autoOutpostWaitTime.get().in(Units.Seconds)))
                        .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                                        driveSubsystem, robotPositioning, "Outpost to Right Hub", false)
                                .andThen(shootCommand))));
    }

    public Command getAutoRoutine() {
        return switch (autoRoutinesChooser.getSelected()) {
            case CENTER_AND_SCORE -> buildCenterAuto();
            case SCORE_LEFT_TO_HUB -> buildScoreLeftAuto();
            case SCORE_RIGHT_TO_HUB -> buildScoreRightAuto();
            case SCORE_LEFT_AND_COLLECT_DEPOT -> buildScoreLeftAndDepot();
            case SCORE_LEFT_AND_COLLECT_NEUTRAL_ZONE -> buildScoreLeftAndNeutral();
            case SCORE_RIGHT_AND_COLLECT_NEUTRAL_ZONE -> buildScoreRightAndNeutral();
            case SCORE_RIGHT_AND_COLLECT_OUTPOST -> buildScoreRightAndOutpost();
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
