package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MoPrefs;
import java.io.File;
import java.util.function.Supplier;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

    private final File directory = new File(Filesystem.getDeployDirectory(), "swerve");
    private final SwerveDrive swerveDrive;

    public DriveSubsystem() {
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
        try {
            swerveDrive = new SwerveParser(directory)
                    .createSwerveDrive(MoPrefs.swerveMaxPossibleSpeed.get().in(Units.MetersPerSecond));
        } catch (Exception e) {
            throw new RuntimeException(e);
        }

        MoPrefs.swerveMaxPossibleSpeed.subscribe(speed -> swerveDrive.setMaximumAttainableSpeeds(
                speed.in(Units.MetersPerSecond),
                MoPrefs.swerveMaxPossibleSpin.get().in(Units.RadiansPerSecond)));
        MoPrefs.swerveMaxPossibleSpin.subscribe(spin -> swerveDrive.setMaximumAttainableSpeeds(
                MoPrefs.swerveMaxPossibleSpeed.get().in(Units.MetersPerSecond), spin.in(Units.RadiansPerSecond)));
        MoPrefs.swerveMaxAllowedSpeed.subscribe(speed -> swerveDrive.setMaximumAllowableSpeeds(
                speed.in(Units.MetersPerSecond),
                MoPrefs.swerveMaxAllowedSpin.get().in(Units.RadiansPerSecond)));
        MoPrefs.swerveMaxAllowedSpin.subscribe(spin -> swerveDrive.setMaximumAllowableSpeeds(
                MoPrefs.swerveMaxAllowedSpeed.get().in(Units.MetersPerSecond), spin.in(Units.RadiansPerSecond)));
    }

    public void driveFieldOriented(ChassisSpeeds velocity) {
        swerveDrive.driveFieldOriented(velocity);
    }

    public Command resetFieldOrientedFwd() {
        return runOnce(() -> swerveDrive.setGyro(Rotation3d.kZero));
    }

    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
        return run(() -> {
            swerveDrive.driveFieldOriented(velocity.get());
        });
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
