package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.util.NTHelpers;
import java.util.function.Supplier;
import swervelib.SwerveDrive;

public class RobotPositioning {
    private static final double STATIONARY_CUTOFF = 1e-2;
    private static final double TURRET_ANGLE_BUFFER_TIME = 2.0; // seconds

    private final SwerveDrive swerveDrive;
    private final Supplier<TurretSubsystem.TimestampedEncoderReading> turretYawSupplier;
    private final Supplier<AngularVelocity> turretYawRateSupplier;

    private final BooleanEntry useMT2;
    private final BooleanEntry useTurretLimelight;
    private final BooleanEntry useStationaryLimelight;

    private final BooleanEntry hasInitialPosition;

    private final TimeInterpolatableBuffer<Rotation2d> turretYawBuffer =
            TimeInterpolatableBuffer.createBuffer(TURRET_ANGLE_BUFFER_TIME);

    // TODO - maybe we can be smarter about this?
    // https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/a875de4792279070e4fcfe990adba8719ec91d5b/src/main/java/org/littletonrobotics/frc2026/subsystems/vision/Vision.java#L210
    private static final Vector<N3> MT1_VISION_STDDEVS = VecBuilder.fill(0.7, 0.7, 9999999);
    private static final Vector<N3> MT2_VISION_STDDEVS = VecBuilder.fill(0.5, 0.5, 9999999);

    private MutAngularVelocity turretAngularVelocity = Units.RPM.mutable(0);

    private final StructPublisher<Pose2d> robotPosePublisher;
    private final StructPublisher<Pose2d> turretLLAprilTagsPublisher;
    private final StructPublisher<Pose2d> turretLLCalculatedPositionPublisher;
    private final DoubleArrayPublisher turretLLTimestampPublisher;
    private final StructPublisher<Pose2d> stationaryLLAprilTagsPublisher;
    private final DoubleArrayPublisher stationaryLLTimestampPublisher;
    private final DoubleArrayPublisher turretPosPublisher;
    private final DoublePublisher turretPosLatencyPublisher;

    public RobotPositioning(
            SwerveDrive swerveDrive,
            Supplier<TurretSubsystem.TimestampedEncoderReading> turretYawSupplier,
            Supplier<AngularVelocity> turretYawRateSupplier) {
        this.swerveDrive = swerveDrive;
        this.turretYawSupplier = turretYawSupplier;
        this.turretYawRateSupplier = turretYawRateSupplier;

        var table = NTHelpers.getTable("odometry");
        useMT2 = NTHelpers.getBooleanEntry(table, "Use MT2", true);
        useTurretLimelight = NTHelpers.getBooleanEntry(table, "Use turret limelight", true);
        useStationaryLimelight = NTHelpers.getBooleanEntry(table, "Use stationary limelight", true);
        hasInitialPosition = NTHelpers.getBooleanEntry(table, "Has initial position", false);

        // Disable limelight to robot transform on turret camera; we will do this ourselves with a more accurate turret
        // pose estimation
        LimelightHelpers.setCameraPose_RobotSpace(Constants.TURRET_LIMELIGHT_NAME, 0, 0, 0.541193, 0, 15, 0);

        LimelightHelpers.setCameraPose_RobotSpace(
                Constants.STATIONARY_LIMELIGHT_NAME, -0.328071, 0.031750, 0.295345, 0, 10, 0);

        robotPosePublisher = table.getStructTopic("robot pose", Pose2d.struct).publish();
        turretLLAprilTagsPublisher =
                table.getStructTopic("turret limelight tags", Pose2d.struct).publish();
        turretLLTimestampPublisher =
                table.getDoubleArrayTopic("turret limelight pose timestamp").publish();
        turretLLCalculatedPositionPublisher = table.getStructTopic(
                        "turret limelight calculated position", Pose2d.struct)
                .publish();
        stationaryLLAprilTagsPublisher =
                table.getStructTopic("stationary limelight tags", Pose2d.struct).publish();
        stationaryLLTimestampPublisher =
                table.getDoubleArrayTopic("stationary limelight pose timestamp").publish();
        turretPosPublisher =
                table.getDoubleArrayTopic("turret position measurements").publish();
        turretPosLatencyPublisher =
                table.getDoubleTopic("turret position measure latency").publish();

        swerveDrive.stopOdometryThread();
    }

    public Pose2d getRobotPose() {
        return swerveDrive.getPose();
    }

    /**
     * Get the field-relative velocity of the robot.
     */
    public ChassisSpeeds getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Get the robot-relative velocity of the robot.
     */
    public ChassisSpeeds getRobotVelocity() {
        return swerveDrive.getRobotVelocity();
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

    private LimelightHelpers.PoseEstimate getPoseEstimateMT2(String limelightName, AngularVelocity angularVelocity) {
        double gyroRateDegreesPerSecond = angularVelocity.in(DegreesPerSecond);

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

    /**
     * Gets a {@link Transform3d} to convert a Pose3d from the limelight's position to the robot's position.
     * <p>
     * This is why we need to do the turret pose transformation ourselves.
     * The turret pose will be constantly changing as it tracks the target, and we need to make sure to apply the
     * reverse transformation for the turret's pose at the instant when the vision measurement was captured.
     * Otherwise we'd apply the wrong reverse transformation and end up with an incorrect robot position.
     * If we just asked the limelight do the transformation, there'd be a delay between when the turret pose was
     * calculated and uploaded to when the vision measurement was captured.
     * By performing the calculation ourselves, we can interpolate between measured turret poses to find the
     * exact turret pose at the exact time the vision measurement was captured.
     * <p>
     * Adapted from
     * https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272#p-3831134-getting-dizzy-3
     */
    private Transform2d getTurretLimelightToRobot(double timestampSeconds) {
        var sample = turretYawBuffer.getSample(timestampSeconds);
        if (sample.isEmpty()) {
            return null;
        }
        var robotToLimelight = TurretSubsystem.robotToTurret
                .plus(new Transform2d(Translation2d.kZero, sample.get()))
                .plus(TurretSubsystem.turretToCamera);

        return robotToLimelight.inverse();
    }

    public void resetOdometry(Pose2d robotPose) {
        swerveDrive.resetOdometry(robotPose);
        hasInitialPosition.set(true);
    }

    private void addVisionMeasurement(Pose2d measuredPose, double timestampSeconds, Vector<N3> stdDevs) {
        if (hasInitialPosition.get() == false) {
            resetOdometry(measuredPose);
        } else {
            swerveDrive.addVisionMeasurement(measuredPose, timestampSeconds, stdDevs);
        }
    }

    private void addTurretVisionMeasurements() {
        if (useTurretLimelight.get() == false) {
            return;
        }

        var turretPosition = swerveDrive
                .getOdometryHeading()
                .plus(Rotation2d.fromRadians(turretYawSupplier.get().value().in(Units.Radians)));
        var turretVelocity = turretAngularVelocity.mut_replace(
                swerveDrive.getRobotVelocity().omegaRadiansPerSecond
                        + turretYawRateSupplier.get().in(Units.RadiansPerSecond),
                Units.RadiansPerSecond);

        LimelightHelpers.PoseEstimate poseEstimate = null;
        Vector<N3> visionStdDevs = null;

        // Can't use MT2 for the turret limelight, because the MT2 algorithm assumes it has an accurate, up-to-date
        // measure of the limelight's rotation. However, the whole point of doing limelight-to-robot transform
        // calculation on the RIO is that it's impossible to keep the limelight's set pose up-to-date since the turret
        // moves so frequently and so fast.
        poseEstimate = getPoseEstimateMT1(Constants.TURRET_LIMELIGHT_NAME);
        visionStdDevs = MT1_VISION_STDDEVS;

        if (poseEstimate == null) {
            return;
        }

        turretLLTimestampPublisher.set(
                new double[] {poseEstimate.timestampSeconds, Timer.getFPGATimestamp() - poseEstimate.timestampSeconds});
        turretLLAprilTagsPublisher.set(poseEstimate.pose);

        var turretLimelightTransform = getTurretLimelightToRobot(poseEstimate.timestampSeconds);
        if (turretLimelightTransform == null) {
            return;
        }

        var robotPose = poseEstimate.pose.transformBy(turretLimelightTransform);

        addVisionMeasurement(robotPose, poseEstimate.timestampSeconds, visionStdDevs);

        turretLLCalculatedPositionPublisher.set(robotPose);
    }

    private void addStationaryVisionMeasurements() {
        if (useStationaryLimelight.get() == false) {
            return;
        }

        // The stationary limelight is assumed to be the LL2, which cannot return accurate measurements while moving.
        var robotVelocity = swerveDrive.getRobotVelocity();
        if (Math.abs(robotVelocity.vxMetersPerSecond) > STATIONARY_CUTOFF
                || Math.abs(robotVelocity.vyMetersPerSecond) > STATIONARY_CUTOFF
                || Math.abs(robotVelocity.omegaRadiansPerSecond) > STATIONARY_CUTOFF) {
            return;
        }

        LimelightHelpers.PoseEstimate poseEstimate = null;
        Vector<N3> visionStdDevs = null;
        if (useMT2.get() && hasInitialPosition.get()) {
            poseEstimate = getPoseEstimateMT2(
                    Constants.STATIONARY_LIMELIGHT_NAME,
                    Units.RadiansPerSecond.of(robotVelocity.omegaRadiansPerSecond));
            visionStdDevs = MT2_VISION_STDDEVS;
        } else {
            poseEstimate = getPoseEstimateMT1(Constants.STATIONARY_LIMELIGHT_NAME);
            visionStdDevs = MT1_VISION_STDDEVS;
        }

        if (poseEstimate == null) {
            return;
        }

        addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, visionStdDevs);

        stationaryLLTimestampPublisher.set(
                new double[] {poseEstimate.timestampSeconds, Timer.getFPGATimestamp() - poseEstimate.timestampSeconds});
        stationaryLLAprilTagsPublisher.set(poseEstimate.pose);
    }

    private void addVisionMeasurements() {
        addTurretVisionMeasurements();
        addStationaryVisionMeasurements();
    }

    public void update() {

        double estimatedHeadingDegrees = swerveDrive.getOdometryHeading().getDegrees();
        LimelightHelpers.SetRobotOrientation(
                Constants.STATIONARY_LIMELIGHT_NAME, estimatedHeadingDegrees, 0, 0, 0, 0, 0);
        LimelightHelpers.SetRobotOrientation(Constants.TURRET_LIMELIGHT_NAME, estimatedHeadingDegrees, 0, 0, 0, 0, 0);

        var encoderReading = turretYawSupplier.get();
        turretYawBuffer.addSample(
                encoderReading.timestamp(),
                Rotation2d.fromRadians(encoderReading.value().in(Units.Radians)));
        turretPosPublisher.set(new double[] {encoderReading.value().in(Units.Degrees), encoderReading.timestamp()});
        turretPosLatencyPublisher.set(Timer.getFPGATimestamp() - encoderReading.timestamp());

        swerveDrive.updateOdometry();
        addVisionMeasurements();

        robotPosePublisher.set(getRobotPose());
    }
}
