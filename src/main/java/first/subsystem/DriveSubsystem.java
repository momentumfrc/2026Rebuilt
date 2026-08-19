package first.subsystem;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import first.MoPrefs;
import first.input.MoInput;
import first.molib.NTHelpers;
import first.molib.prefs.MoPrefsUtils;
import first.util.MoSwerveInputStream;
import first.util.MutablePIDConstants;
import java.io.File;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.kinematics.ChassisSpeeds;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.system.Filesystem;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Current;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
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

    private boolean boostCurrentLimits = false;

    private final SendableChooser<DriveMode> driveModeChooser =
            NTHelpers.enumToChooser(DriveMode.class, DriveMode.VELOCITY_HEADING);

    private final DoublePublisher omegaSpeed;
    private final BooleanPublisher boostCurrentLimitsPublisher;

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
        boostCurrentLimitsPublisher =
                table.getBooleanTopic("Boost Current Limits").publish();

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

    public void stop() {
        swerveDrive.drive(new ChassisSpeeds());
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    private Supplier<Supplier<ChassisSpeeds>> setupDriveModes(Supplier<MoInput> inputSupplier) {
        var swerveInputStreamBase = MoSwerveInputStream.of(
                        swerveDrive,
                        () -> inputSupplier.get().getDriveMoveXRequest(),
                        () -> inputSupplier.get().getDriveMoveYRequest())
                .allianceRelativeControl(true)
                .cubeTranslationControllerAxis(() -> MoPrefs.inputTranslationCubed.get())
                .cubeRotationControllerAxis(() -> MoPrefs.inputRotationCubed.get());

        swerveInputStreamBase.scaleTranslation(
                () -> inputSupplier.get().getRunIntake() ? MoPrefs.driveIntakingSlowSpeed.get() : 1);

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

    public boolean isBoosted() {
        return boostCurrentLimits;
    }

    public void toggleBoostCurrentLimits() {
        if (boostCurrentLimits) {
            boostCurrentLimits = false;
            restoreCurrentLimits();
        } else {
            boostCurrentLimits = true;
            overrideCurrentLimits(
                    (Current) MoPrefs.boostDriveMotorLimit.get(), (Current) MoPrefs.boostSteerMotorLimit.get());
        }
    }

    public void overrideCurrentLimits(Current driveLimit, Current steerLimit) {
        final int driveLimitAmps = (int) driveLimit.in(Units.Amps);
        final int steerLimitAmps = (int) steerLimit.in(Units.Amps);

        final SwerveModule[] swerveModules = swerveDrive.getModules();

        new Thread(() -> {
                    for (var module : swerveModules) {
                        module.getDriveMotor().setCurrentLimit(driveLimitAmps);
                        module.getAngleMotor().setCurrentLimit(steerLimitAmps);
                    }
                })
                .start();
    }

    public void restoreCurrentLimits() {
        final SwerveModule[] swerveModules = swerveDrive.getModules();

        new Thread(() -> {
                    for (var module : swerveModules) {
                        module.getDriveMotor()
                                .setCurrentLimit(module.configuration.physicalCharacteristics.driveMotorCurrentLimit);
                        module.getAngleMotor()
                                .setCurrentLimit(module.configuration.physicalCharacteristics.angleMotorCurrentLimit);
                    }
                })
                .start();
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
        boostCurrentLimitsPublisher.set(boostCurrentLimits);
    }
}
