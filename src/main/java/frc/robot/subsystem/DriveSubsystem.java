package frc.robot.subsystem;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MoPrefs;
import frc.robot.molib.prefs.MoPrefsUtils;
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

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return swerveDrive.getRobotVelocity();
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }
}
