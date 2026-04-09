package frc.robot.commands.auto;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.MoPrefs;
import frc.robot.RobotPositioning;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ZeroHoodCommand;
import frc.robot.commands.intake.RollerCommands;
import frc.robot.commands.intake.WristCommands;
import frc.robot.molib.NTHelpers;
import frc.robot.molib.Utils;
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
    private static final PathConstraints AUTO_PATH_CONSTRAINTS = new PathConstraints(
            Units.MetersPerSecond.of(2),
            Units.MetersPerSecondPerSecond.of(2),
            Units.RotationsPerSecond.of(2),
            Units.RotationsPerSecondPerSecond.of(2));

    private enum AutoChoices {
        SHOOT
    } // probably won't need this, leaving it here just in case we do for whatever reason

    private enum ShootAutoRoutines {
        SHOOT_ONLY,
        SHOOT_THEN_BACKUP,
        CENTER_AND_SCORE,
        LEFT_COLLECT_NEUTRAL_TO_HUB,
        RIGHT_COLLECT_NEUTRAL_TO_HUB,
        SCORE_LEFT_TO_HUB,
        SCORE_RIGHT_TO_HUB,
        SCORE_LEFT_AND_COLLECT_DEPOT,
        SCORE_LEFT_AND_COLLECT_NEUTRAL_ZONE,
        SCORE_RIGHT_AND_COLLECT_NEUTRAL_ZONE,
        SCORE_RIGHT_AND_COLLECT_OUTPOST,
        TEST_PATH
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

    private final BooleanEntry enableAutoSwitch;

    private SendableChooser<AutoChoices> autoChoicesChooser = NTHelpers.enumToChooser(AutoChoices.class);
    private SendableChooser<ShootAutoRoutines> autoRoutinesChooser = NTHelpers.enumToChooser(ShootAutoRoutines.class);
    private BooleanEntry assumeRobotPose;

    private DoubleEntry backupDistance;

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

        var autoTable = NTHelpers.getTable("Auto");
        enableAutoSwitch = NTHelpers.getBooleanEntry(autoTable, "Run Auto?", true);

        NTHelpers.publishSendable(autoTable, "Which Auto?", autoChoicesChooser);
        NTHelpers.publishSendable(autoTable, "Which Routine?", autoRoutinesChooser);
        assumeRobotPose = NTHelpers.getBooleanEntry(autoTable, "Assume Robot Position?", false);

        backupDistance = NTHelpers.getDoubleEntry(autoTable, "Auto Backup Distance (m)", 1);
    }

    public Command getShootCommand() {
        var shootCommand = ShootCommand.getHubShootCommand(
                turretTargeting, kickerSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem);

        var rezeroCommand =
                Commands.either(Commands.none(), new ZeroHoodCommand(hoodSubsystem), hoodSubsystem::hasZero);

        var command = rezeroCommand.andThen(shootCommand
                .alongWith(Commands.defer(() -> Commands.waitUntil(shootCommand::readyToShoot), Collections.emptySet())
                        .andThen(indexerSubsystem.run(indexerSubsystem::run)))
                .withName("AutoShootCommand"));

        return Utils.withTimeoutPref(command, MoPrefs.autoShooterRunTime::get);
    }

    public Command getIntakeCommand() {
        return WristCommands.moveToPositionCommand(intakeWristSubsystem, MoPrefs.intakeWristDeployPosition::get)
                .andThen(Commands.parallel(
                        WristCommands.holdIntakeWristCommand(intakeWristSubsystem, WristCommands.Direction.OUT),
                        RollerCommands.runIntakeRollerCommand(intakeRollerSubsystem)));
    }

    public Command getShootThenBackupCommand() {
        double dx = backupDistance.get();
        if (DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue) {
            dx = -1 * dx;
        }

        var robotPose = robotPositioning.getRobotPose();
        double minX = edu.wpi.first.math.util.Units.inchesToMeters(36 / 2);
        double maxX = edu.wpi.first.math.util.Units.inchesToMeters(650.92 - (36 / 2));
        double newX = robotPose.getX() + dx;
        newX = Math.min(newX, maxX);
        newX = Math.max(newX, minX);
        var targetTrans = new Translation2d(newX, robotPose.getY());

        Rotation2d directionOfTravel =
                targetTrans.minus(robotPose.getTranslation()).getAngle();
        var waypoints = PathPlannerPath.waypointsFromPoses(
                new Pose2d(robotPose.getTranslation(), directionOfTravel), new Pose2d(targetTrans, directionOfTravel));

        var path = new PathPlannerPath(
                waypoints, AUTO_PATH_CONSTRAINTS, null, new GoalEndState(0, robotPose.getRotation()));
        path.preventFlipping = true;

        return getShootCommand()
                .andThen(AutoPathPlannerCommands.getFollowPathCommand(driveSubsystem, robotPositioning, path));
    }

    // We shoot the fuel, and then we move out of the way in case another alliance member needs to move there (unlikely
    // but just incase)
    public Command buildCenterAuto() {
        return getShootCommand()
                .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Center Forward", assumeRobotPose.get()));
    }

    public Command buildLeftCollectAndScoreAuto() {
        return AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Left Start to Left Neutral Zone", assumeRobotPose.get())
                .andThen(Commands.deadline(
                        AutoPathPlannerCommands.getFollowPathCommand(
                                driveSubsystem, robotPositioning, "Left Neutral Zone to Left Hub", false),
                        getIntakeCommand()))
                .andThen(getShootCommand());
    }

    public Command buildRightCollectAndScoreAuto() {
        return AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Right Start to Right Neutral Zone", assumeRobotPose.get())
                .andThen(Commands.deadline(
                        AutoPathPlannerCommands.getFollowPathCommand(
                                driveSubsystem, robotPositioning, "Right Neutral Zone to Right Hub", false),
                        getIntakeCommand()))
                .andThen(getShootCommand());
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
                        getIntakeCommand()))
                .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Depot to Left Hub", false))
                .andThen(getShootCommand());
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
                        driveSubsystem, robotPositioning, "Right Start to Right Hub", assumeRobotPose.get())
                .andThen(getShootCommand())
                .andThen(Commands.deadline(
                        AutoPathPlannerCommands.getFollowPathCommand(
                                driveSubsystem, robotPositioning, "Right Hub to Outpost", false),
                        getIntakeCommand()))
                .andThen(Commands.waitSeconds(MoPrefs.autoOutpostWaitTime.get().in(Units.Seconds)))
                .andThen(AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "Outpost to Right Hub", false))
                .andThen(getShootCommand());
    }

    public Command getAutoRoutine() {
        return switch (autoRoutinesChooser.getSelected()) {
            case SHOOT_ONLY -> getShootCommand();
            case SHOOT_THEN_BACKUP -> getShootThenBackupCommand();
            case CENTER_AND_SCORE -> buildCenterAuto();
            case LEFT_COLLECT_NEUTRAL_TO_HUB -> buildLeftCollectAndScoreAuto();
            case RIGHT_COLLECT_NEUTRAL_TO_HUB -> buildRightCollectAndScoreAuto();
            case SCORE_LEFT_TO_HUB -> buildScoreLeftAuto();
            case SCORE_RIGHT_TO_HUB -> buildScoreRightAuto();
            case SCORE_LEFT_AND_COLLECT_DEPOT -> buildScoreLeftAndDepot();
            case SCORE_LEFT_AND_COLLECT_NEUTRAL_ZONE -> buildScoreLeftAndNeutral();
            case SCORE_RIGHT_AND_COLLECT_NEUTRAL_ZONE -> buildScoreRightAndNeutral();
            case SCORE_RIGHT_AND_COLLECT_OUTPOST -> buildScoreRightAndOutpost();
            case TEST_PATH ->
                AutoPathPlannerCommands.getFollowPathCommand(
                        driveSubsystem, robotPositioning, "TEST", assumeRobotPose.get());
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
