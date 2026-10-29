package first.robot;

import static org.wpilib.units.Units.DegreesPerSecond;

import first.robot.molib.NTHelpers;
import first.robot.util.LimelightHelpers;
import java.util.Set;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.linalg.Vector;
import org.wpilib.math.numbers.N3;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.DoubleArrayPublisher;
import org.wpilib.networktables.StructPublisher;
import org.wpilib.system.Timer;
import org.wpilib.units.Units;
import org.wpilib.units.measure.AngularVelocity;
import swervelib.SwerveDrive;

public class RobotPositioning {
    private static final double STATIONARY_CUTOFF = 1e-2;

    private static final Set<Integer> DISALLOWED_TAGS = Set.of(1, 12, 22, 23);

    private final SwerveDrive swerveDrive;

    private final BooleanEntry useMT2;
    private final BooleanEntry useTurretLimelight;
    private final BooleanEntry useStationaryLimelight;

    private final BooleanEntry hasInitialPosition;

    // TODO - maybe we can be smarter about this?
    // https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/a875de4792279070e4fcfe990adba8719ec91d5b/src/main/java/org/littletonrobotics/frc2026/subsystems/vision/Vision.java#L210
    private static final Vector<N3> MT1_VISION_STDDEVS = VecBuilder.fill(0.7, 0.7, 9999999);
    private static final Vector<N3> MT2_VISION_STDDEVS = VecBuilder.fill(0.5, 0.5, 9999999);

    private final StructPublisher<Pose2d> robotPosePublisher;
    private final StructPublisher<Pose2d> turretLLCalculatedPositionPublisher;
    private final DoubleArrayPublisher turretLLTimestampPublisher;
    private final StructPublisher<Pose2d> stationaryLLAprilTagsPublisher;
    private final DoubleArrayPublisher stationaryLLTimestampPublisher;

    public RobotPositioning(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

        var table = NTHelpers.getTable("odometry");
        useMT2 = NTHelpers.getBooleanEntry(table, "Use MT2", true);
        useTurretLimelight = NTHelpers.getBooleanEntry(table, "Use turret limelight", true);
        useStationaryLimelight = NTHelpers.getBooleanEntry(table, "Use stationary limelight", true);
        hasInitialPosition = NTHelpers.getBooleanEntry(table, "Has initial position", false);

        LimelightHelpers.setCameraPose_RobotSpace(
                Constants.TURRET_LIMELIGHT_NAME, 0.0345140, 0.002023, 0.541193, 0, 15, 9.2);
        LimelightHelpers.setCameraPose_RobotSpace(
                Constants.STATIONARY_LIMELIGHT_NAME, -0.328071, 0.031750, 0.295345, 0, 10, 180);

        int[] allowedTagIds = Fields.FRC_2026_REBUILT_WELDED.loadField().tags.stream()
                .mapToInt(tag -> tag.getID())
                .filter(tag -> DISALLOWED_TAGS.contains(tag) == false)
                .toArray();
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.TURRET_LIMELIGHT_NAME, allowedTagIds);
        LimelightHelpers.SetFiducialIDFiltersOverride(Constants.STATIONARY_LIMELIGHT_NAME, allowedTagIds);

        robotPosePublisher = table.getStructTopic("robot pose", Pose2d.struct).publish();
        turretLLTimestampPublisher =
                table.getDoubleArrayTopic("turret limelight pose timestamp").publish();
        turretLLCalculatedPositionPublisher = table.getStructTopic(
                        "turret limelight calculated position", Pose2d.struct)
                .publish();
        stationaryLLAprilTagsPublisher =
                table.getStructTopic("stationary limelight tags", Pose2d.struct).publish();
        stationaryLLTimestampPublisher =
                table.getDoubleArrayTopic("stationary limelight pose timestamp").publish();

        swerveDrive.stopOdometryThread();
    }

    public Pose2d getRobotPose() {
        return swerveDrive.getPose();
    }

    public boolean hasInitialPosition() {
        return hasInitialPosition.get();
    }

    /**
     * Get the field-relative velocity of the robot.
     */
    public ChassisVelocities getFieldVelocity() {
        return swerveDrive.getFieldVelocity();
    }

    /**
     * Get the robot-relative velocity of the robot.
     */
    public ChassisVelocities getRobotVelocity() {
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

        LimelightHelpers.PoseEstimate poseEstimate = null;
        Vector<N3> visionStdDevs = null;
        var robotVelocity = swerveDrive.getRobotVelocity();

        if (useMT2.get() && hasInitialPosition.get()) {
            poseEstimate =
                    getPoseEstimateMT2(Constants.TURRET_LIMELIGHT_NAME, Units.RadiansPerSecond.of(robotVelocity.omega));
            visionStdDevs = MT2_VISION_STDDEVS;
        } else {
            poseEstimate = getPoseEstimateMT1(Constants.TURRET_LIMELIGHT_NAME);
            visionStdDevs = MT1_VISION_STDDEVS;
        }

        if (poseEstimate == null) {
            return;
        }

        addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, visionStdDevs);

        turretLLTimestampPublisher.set(new double[] {
            poseEstimate.timestampSeconds, Timer.getMonotonicTimestamp() - poseEstimate.timestampSeconds
        });
        turretLLCalculatedPositionPublisher.set(poseEstimate.pose);
    }

    private void addStationaryVisionMeasurements() {
        if (useStationaryLimelight.get() == false) {
            return;
        }

        // The stationary limelight is assumed to be the LL2, which cannot return accurate measurements while moving.
        var robotVelocity = swerveDrive.getRobotVelocity();
        /*
        if (Math.abs(robotVelocity.vx) > STATIONARY_CUTOFF
                || Math.abs(robotVelocity.vy) > STATIONARY_CUTOFF
                || Math.abs(robotVelocity.omega) > STATIONARY_CUTOFF) {
            return;
        } */

        LimelightHelpers.PoseEstimate poseEstimate = null;
        Vector<N3> visionStdDevs = null;
        if (useMT2.get() && hasInitialPosition.get()) {
            poseEstimate = getPoseEstimateMT2(
                    Constants.STATIONARY_LIMELIGHT_NAME, Units.RadiansPerSecond.of(robotVelocity.omega));
            visionStdDevs = MT2_VISION_STDDEVS;
        } else {
            poseEstimate = getPoseEstimateMT1(Constants.STATIONARY_LIMELIGHT_NAME);
            visionStdDevs = MT1_VISION_STDDEVS;
        }

        if (poseEstimate == null) {
            return;
        }

        addVisionMeasurement(poseEstimate.pose, poseEstimate.timestampSeconds, visionStdDevs);

        stationaryLLTimestampPublisher.set(new double[] {
            poseEstimate.timestampSeconds, Timer.getMonotonicTimestamp() - poseEstimate.timestampSeconds
        });
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

        swerveDrive.updateOdometry();
        addVisionMeasurements();
        robotPosePublisher.set(getRobotPose());
    }
}
