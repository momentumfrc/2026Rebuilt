package frc.robot.subsystem;

import static edu.wpi.first.math.util.Units.radiansToDegrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MoPrefs;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTHelpers;
import java.util.function.Supplier;
import swervelib.SwerveDrive;

public class PositioningSubsystem extends SubsystemBase {
    private static final String TURRET_LL_NAME = "limelight";

    private final SwerveDrive swerveDrive;
    private final Supplier<Pose3d> turretLimelightPoseSupplier;

    private final BooleanEntry useMT2;
    private final BooleanEntry turretPoseIsUpToDate;

    private Pose3d lastTurretPose = null;
    private Timer turretUpdateTimer;

    public PositioningSubsystem(SwerveDrive swerveDrive, Supplier<Pose3d> turretLimelightPoseSupplier) {
        this.swerveDrive = swerveDrive;
        this.turretLimelightPoseSupplier = turretLimelightPoseSupplier;

        var table = NTHelpers.getTable("odometry");
        useMT2 = NTHelpers.getBooleanEntry(table, "Use MT2", true);
        turretPoseIsUpToDate = NTHelpers.getBooleanEntry(table, "Turret pose up to date", false);
    }

    private boolean checkTurretPoseIsUpToDate() {
        var currentTurretPose = turretLimelightPoseSupplier.get();

        if (lastTurretPose == null || currentTurretPose.equals(lastTurretPose) == false) {
            var cameraTranslation = currentTurretPose.getTranslation();
            var cameraRotation = currentTurretPose.getRotation();

            LimelightHelpers.setCameraPose_RobotSpace(
                    TURRET_LL_NAME,
                    cameraTranslation.getX(),
                    cameraTranslation.getY(),
                    cameraTranslation.getZ(),
                    radiansToDegrees(cameraRotation.getX()),
                    radiansToDegrees(cameraRotation.getY()),
                    radiansToDegrees(cameraRotation.getZ()));

            lastTurretPose = currentTurretPose;
            turretUpdateTimer.restart();
        }

        boolean upToDate = turretUpdateTimer.hasElapsed(MoPrefs.limelightPoseRefreshDelay.get());
        turretPoseIsUpToDate.set(upToDate);
        return upToDate;
    }

    // Logic adapted from https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-swerve-pose-estimation
    private void updatePoseEstimateMT1() {
        var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(TURRET_LL_NAME);
        if (mt1.tagCount == 0) {
            return;
        }
        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > 0.7) {
                return;
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
                return;
            }
        }

        swerveDrive.addVisionMeasurement(mt1.pose, mt1.timestampSeconds, VecBuilder.fill(0.5, 0.5, 9999999));
    }

    private void updatePoseEstimateMT2() {
        double estimatedHeadingDegrees = swerveDrive.getYaw().getDegrees();
        double gyroRateDegreesPerSecond =
                swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);

        LimelightHelpers.SetRobotOrientation(TURRET_LL_NAME, estimatedHeadingDegrees, 0, 0, 0, 0, 0);
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(TURRET_LL_NAME);

        if (Math.abs(gyroRateDegreesPerSecond) > 720) {
            return;
        }

        if (mt2.tagCount == 0) {
            return;
        }

        swerveDrive.addVisionMeasurement(mt2.pose, mt2.timestampSeconds, VecBuilder.fill(0.7, 0.7, 9999999));
    }

    public void addVisionMeasurements() {
        if (checkTurretPoseIsUpToDate() == false) {
            return;
        }

        if (useMT2.get()) {
            updatePoseEstimateMT2();
        } else {
            updatePoseEstimateMT1();
        }
    }

    @Override
    public void periodic() {
        addVisionMeasurements();
    }
}
