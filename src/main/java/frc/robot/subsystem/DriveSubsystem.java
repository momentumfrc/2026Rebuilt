package frc.robot.subsystem;

import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MoPrefs;
import frc.robot.input.MoInput;
import frc.robot.molib.prefs.MoPrefsUtils;
import frc.robot.util.NTHelpers;
import java.io.File;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {
    private final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    private final SwerveDrive swerveDrive;

    private enum DriveMode {
        VELOCITY_HEADING,
        ABSOLUTE_HEADING
    };

    private final SendableChooser<DriveMode> driveModeChooser =
            NTHelpers.enumToChooser(DriveMode.class, DriveMode.VELOCITY_HEADING);

    public DriveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory)
                    .createSwerveDrive(MoPrefs.swerveMaxPossibleSpeed.get().in(Units.MetersPerSecond));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        MoPrefsUtils.multiSubscribe(
                MoPrefs.swerveMaxPossibleSpeed,
                MoPrefs.swerveMaxPossibleSpin,
                (speed, spin) -> swerveDrive.setMaximumAttainableSpeeds(
                        speed.in(Units.MetersPerSecond), spin.in(Units.RadiansPerSecond)),
                true);
        MoPrefsUtils.multiSubscribe(
                MoPrefs.swerveMaxAllowedSpeed,
                MoPrefs.swerveMaxAllowedSpin,
                (speed, spin) -> swerveDrive.setMaximumAllowableSpeeds(
                        speed.in(Units.MetersPerSecond), spin.in(Units.RadiansPerSecond)),
                true);

        var table = NTHelpers.getTable("drive");
        NTHelpers.publishSendable(table, "Drive Mode", driveModeChooser);
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command resetFieldOrientedFwd() {
        return runOnce(() -> swerveDrive.zeroGyro());
    }

    public Command driveFieldOriented(Supplier<Supplier<ChassisSpeeds>> inputSupplier) {
        return run(() -> {
            swerveDrive.driveFieldOriented(inputSupplier.get().get());
        });
    }

    public void driveRobotRelativeSpeeds(ChassisSpeeds chassisSpeeds, DriveFeedforwards driveFeedforwards) {
        swerveDrive.drive(
                chassisSpeeds,
                swerveDrive.kinematics.toSwerveModuleStates(chassisSpeeds),
                driveFeedforwards.linearForces());
    }

    public void autoDefaultCommand(double xRequest, double yRequest) {
        SwerveInputStream.of(swerveDrive, () -> xRequest, () -> yRequest);
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    private Supplier<Supplier<ChassisSpeeds>> setupDriveModes(Supplier<MoInput> inputSupplier) {
        var swerveInputStreamBase = SwerveInputStream.of(
                        swerveDrive,
                        () -> inputSupplier.get().getDriveMoveXRequest(),
                        () -> inputSupplier.get().getDriveMoveYRequest())
                .allianceRelativeControl(true)
                .cubeTranslationControllerAxis(() -> MoPrefs.inputTranslationCubed.get())
                .cubeRotationControllerAxis(() -> MoPrefs.inputRotationCubed.get());

        var driveAngularVelocity = swerveInputStreamBase
                .copy()
                .withControllerRotationAxis(() -> inputSupplier.get().getDriveTurnRequest());

        var driveHeading = swerveInputStreamBase
                .copy()
                .withControllerHeadingAxis(
                        () -> inputSupplier.get().getDriveHeadingXRequest(),
                        () -> inputSupplier.get().getDriveHeadingYRequest());

        MoPrefs.inputDeadband.subscribe(
                deadband -> {
                    driveAngularVelocity.deadband(deadband);
                    driveHeading.deadband(deadband);
                },
                true);
        MoPrefs.inputTranslationScale.subscribe(
                scale -> {
                    driveAngularVelocity.scaleTranslation(scale);
                    driveHeading.scaleTranslation(scale);
                },
                true);
        MoPrefs.inputRotationScale.subscribe(scale -> {
            driveAngularVelocity.scaleRotation(scale);
            driveHeading.scaleRotation(scale);
        });

        return () -> switch (driveModeChooser.getSelected()) {
            case VELOCITY_HEADING -> driveAngularVelocity;
            case ABSOLUTE_HEADING -> driveHeading;
        };
    }

    public Command getTeleopDriveCommand(Supplier<MoInput> inputSupplier) {
        return driveFieldOriented(setupDriveModes(inputSupplier));
    }

    public Command lockPose() {
        return run(() -> {
            swerveDrive.lockPose();
        });
    }
}
