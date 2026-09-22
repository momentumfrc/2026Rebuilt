package first.robot.commands.auto;

import first.robot.RobotPositioning;
import first.robot.commands.ShootCommand;
import first.robot.commands.ZeroHoodCommand;
import first.robot.molib.NTHelpers;
import first.robot.shootutils.TurretTargeting;
import first.robot.subsystem.DriveSubsystem;
import first.robot.subsystem.HoodSubsystem;
import first.robot.subsystem.IndexerSubsystem;
import first.robot.subsystem.IntakeRollerSubsystem;
import first.robot.subsystem.IntakeWristSubsystem;
import first.robot.subsystem.KickerSubsystem;
import first.robot.subsystem.ShooterSubsystem;
import first.robot.subsystem.TurretSubsystem;
import java.util.Collections;
import java.util.Set;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Time;

public class AutoChooser {

    private enum ShootAutoRoutines {
        SHOOT_ONLY,
    }

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

    private SendableChooser<ShootAutoRoutines> autoRoutinesChooser = NTHelpers.enumToChooser(ShootAutoRoutines.class);
    private BooleanEntry assumeRobotPose;

    private DoubleEntry backupDistance;

    private final Set<SubsystemBase> proxiedSubsystems;

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

        proxiedSubsystems =
                Set.of(intakeRollerSubsystem, intakeWristSubsystem, shooterSubsystem, hoodSubsystem, kickerSubsystem);

        turretTargeting = new TurretTargeting(robotPositioning);

        var autoTable = NTHelpers.getTable("Auto");
        enableAutoSwitch = NTHelpers.getBooleanEntry(autoTable, "Run Auto?", true);

        NTHelpers.publishSendable(autoTable, "Which Routine?", autoRoutinesChooser);
        assumeRobotPose = NTHelpers.getBooleanEntry(autoTable, "Assume Robot Position?", false);

        backupDistance = NTHelpers.getDoubleEntry(autoTable, "Auto Backup Distance (m)", 1);
    }

    public Command getShootCommand(Time shootTime) {
        var shootCommand = ShootCommand.getHubShootCommand(
                turretTargeting, kickerSubsystem, turretSubsystem, shooterSubsystem, hoodSubsystem);

        var rezeroCommand =
                Commands.either(Commands.none(), new ZeroHoodCommand(hoodSubsystem), hoodSubsystem::hasZero);

        Command command = rezeroCommand.andThen(shootCommand
                .alongWith(Commands.defer(() -> Commands.waitUntil(shootCommand::readyToShoot), Collections.emptySet())
                        .andThen(indexerSubsystem.run(indexerSubsystem::run)))
                .withName("AutoShootCommand"));

        command = Commands.either(
                command,
                Commands.print("Refusing to shoot without an established initial position"),
                robotPositioning::hasInitialPosition);

        return command.asProxy().withTimeout(shootTime);
    }

    public Command getAutoRoutine() {
        return switch (autoRoutinesChooser.getSelected()) {
            case SHOOT_ONLY -> getShootCommand(Units.Seconds.of(8));
        };
    }

    public Command getAutoChooserCommand() {
        if (!enableAutoSwitch.get()) {
            return Commands.print("Auto Disabled");
        }
        var auto = getAutoRoutine();
        for (var subsystem : proxiedSubsystems) {
            if (auto.getRequirements().contains(subsystem)) {
                DriverStationErrors.reportWarning(
                        "Auto command requires illegal subsystem [" + subsystem.getName() + "]", false);
            }
        }

        return auto;
    }
}
