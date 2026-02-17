package frc.robot;

import static edu.wpi.first.math.util.Units.radiansToDegrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTHelpers;
import java.util.function.Supplier;
import swervelib.SwerveDrive;

public class RobotPositioning {
    private static final double STATIONARY_CUTOFF = 1e-2;

    private final SwerveDrive swerveDrive;
    private final Supplier<Pose3d> turretLimelightPoseSupplier;

    private final BooleanEntry useMT2;
    private final BooleanEntry useTurretLimelight;
    private final BooleanEntry useStationaryLimelight;

    private final BooleanEntry turretPoseIsUpToDate;

    private final Timer turretUpdateTimer;
    private Pose3d lastTurretPose = null;

    public RobotPositioning(SwerveDrive swerveDrive, Supplier<Pose3d> turretLimelightPoseSupplier) {
        this.swerveDrive = swerveDrive;
        this.turretLimelightPoseSupplier = turretLimelightPoseSupplier;

        this.turretUpdateTimer = new Timer();

        var table = NTHelpers.getTable("odometry");
        useMT2 = NTHelpers.getBooleanEntry(table, "Use MT2", true);
        useTurretLimelight = NTHelpers.getBooleanEntry(table, "Use turret limelight", true);
        useStationaryLimelight = NTHelpers.getBooleanEntry(table, "Use stationary limelight", true);
        turretPoseIsUpToDate = NTHelpers.getBooleanEntry(table, "Turret pose up to date", false);
    }

    public Pose2d getRobotPose() {
        return swerveDrive.getPose();
    }

    private boolean checkTurretPoseIsUpToDate() {
        var currentTurretPose = turretLimelightPoseSupplier.get();

        if (lastTurretPose == null || currentTurretPose.equals(lastTurretPose) == false) {
            var cameraTranslation = currentTurretPose.getTranslation();
            var cameraRotation = currentTurretPose.getRotation();

            LimelightHelpers.setCameraPose_RobotSpace(
                    Constants.TURRET_LIMELIGHT_NAME,
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
    private LimelightHelpers.PoseEstimate getPoseEstimateMT1(String limelightName) {
        var mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);
        if (mt1 == null) {
            return null;
        }

        if (mt1.tagCount == 0) {
            return null;
        }
        if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
            if (mt1.rawFiducials[0].ambiguity > 0.7) {
                return null;
            }
            if (mt1.rawFiducials[0].distToCamera > 3) {
                return null;
            }
        }

        return mt1;
    }

    private LimelightHelpers.PoseEstimate getPoseEstimateMT2(String limelightName) {
        double estimatedHeadingDegrees = swerveDrive.getYaw().getDegrees();
        double gyroRateDegreesPerSecond =
                swerveDrive.getGyro().getYawAngularVelocity().in(DegreesPerSecond);

        LimelightHelpers.SetRobotOrientation(limelightName, estimatedHeadingDegrees, 0, 0, 0, 0, 0);
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        if (mt2 == null) {
            return null;
        }

        if (Math.abs(gyroRateDegreesPerSecond) > 720) {
            return null;
        }

        if (mt2.tagCount == 0) {
            return null;
        }

        return mt2;
    }

    private void addTurretVisionMeasurements() {
        if (useTurretLimelight.get() == false) {
            return;
        }

        if (checkTurretPoseIsUpToDate() == false) {
            return;
        }

        if (useMT2.get()) {
            var estimate = getPoseEstimateMT2(Constants.TURRET_LIMELIGHT_NAME);
            if (estimate != null) {
                swerveDrive.addVisionMeasurement(
                        estimate.pose, estimate.timestampSeconds, VecBuilder.fill(0.5, 0.5, 9999999));
            }
        } else {
            var estimate = getPoseEstimateMT1(Constants.TURRET_LIMELIGHT_NAME);
            if (estimate != null) {
                swerveDrive.addVisionMeasurement(
                        estimate.pose, estimate.timestampSeconds, VecBuilder.fill(0.7, 0.7, 9999999));
            }
        }
    }

    private void addStationaryVisionMeasurements() {
        if (useStationaryLimelight.get() == false) {
            return;
        }

        // The stationary limelight is assumed to be the LL2, which cannot return accurate measurements while moving.
        var robotVelocity = swerveDrive.getFieldVelocity();
        if (Math.abs(robotVelocity.vxMetersPerSecond) > STATIONARY_CUTOFF
                || Math.abs(robotVelocity.vyMetersPerSecond) > STATIONARY_CUTOFF
                || Math.abs(robotVelocity.omegaRadiansPerSecond) > STATIONARY_CUTOFF) {
            return;
        }

        if (useMT2.get()) {
            var estimate = getPoseEstimateMT2(Constants.STATIONARY_LIMELIGHT_NAME);
            if (estimate != null) {
                swerveDrive.addVisionMeasurement(
                        estimate.pose, estimate.timestampSeconds, VecBuilder.fill(0.5, 0.5, 9999999));
            }
        } else {
            var estimate = getPoseEstimateMT1(Constants.STATIONARY_LIMELIGHT_NAME);
            if (estimate != null) {
                swerveDrive.addVisionMeasurement(
                        estimate.pose, estimate.timestampSeconds, VecBuilder.fill(0.7, 0.7, 9999999));
            }
        }
    }

    public void addVisionMeasurements() {
        addTurretVisionMeasurements();
        addStationaryVisionMeasurements();
    }

    public void update() {
        addVisionMeasurements();
    }
}
