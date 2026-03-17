package frc.robot.subsystem;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MoPrefs;
import frc.robot.input.MoInput;
import frc.robot.molib.NTHelpers;
import frc.robot.molib.prefs.MoPrefsUtils;
import frc.robot.util.MutablePIDConstants;
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

    private final MutablePIDConstants translationPIDConstants = new MutablePIDConstants();
    private final MutablePIDConstants rotationPIDConstants = new MutablePIDConstants();

    private enum DriveMode {
        VELOCITY_HEADING,
        ABSOLUTE_HEADING
    };

    private final SendableChooser<DriveMode> driveModeChooser =
            NTHelpers.enumToChooser(DriveMode.class, DriveMode.VELOCITY_HEADING);

    private final DoublePublisher omegaSpeed;

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

        MoPrefs.enableHeadingCorrection.subscribe(swerveDrive::setHeadingCorrection, true);

        var table = NTHelpers.getTable("drive");
        NTHelpers.publishSendable(table, "Drive Mode", driveModeChooser);
        omegaSpeed = table.getDoubleTopic("Gyro Speed (rad_s)").publish();

        translationPIDConstants.getTuner("PathPlanner Translation PID").safeBuild();
        rotationPIDConstants.getTuner("PathPlanner Rotation PID").safeBuild();
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command resetFieldOrientedFwd() {
        return runOnce(() -> {
            swerveDrive.zeroGyro();

            var alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
            if (alliance == DriverStation.Alliance.Red) {
                swerveDrive.setGyro(new Rotation3d(Rotation2d.k180deg));
            }
            swerveDrive.setGyro(new Rotation3d(Rotation2d.k180deg));
            swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPose().getTranslation(), Rotation2d.k180deg));
        });
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

    public PPHolonomicDriveController driveController() {
        return new PPHolonomicDriveController(
                translationPIDConstants.toImmutable(), rotationPIDConstants.toImmutable());
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
                .headingWhile(true)
                .withControllerHeadingAxis(
                        () -> inputSupplier.get().getDriveHeadingXRequest(),
                        () -> inputSupplier.get().getDriveHeadingYRequest());

        MoPrefs.inputDeadband.subscribe(
                deadband -> {
                    driveAngularVelocity.deadband(deadband);
                    driveHeading.deadband(deadband);
                },
                true);

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

    public void periodic() {
        omegaSpeed.set(swerveDrive.getRobotVelocity().omegaRadiansPerSecond);
    }
}
