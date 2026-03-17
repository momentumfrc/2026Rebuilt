package frc.robot.shootutils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.RobotPositioning;
import frc.robot.molib.NTHelpers;
import frc.robot.subsystem.TurretSubsystem;

/**
 * Logic to perform turret targeting for shoot-on-the-fly.
 * <p>
 * Heavily based on https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272
 */
public final class TurretTargeting {
    private static final int NUM_LOOKAHEAD_ESTIMATION_ITERATIONS = 20;

    private final RobotPositioning positioning;

    public static class TurretSetpoint {
        private final MutAngle goalAngle = Units.Radians.mutable(0);
        private final MutAngularVelocity goalVelocity = Units.RadiansPerSecond.mutable(0);
        private final MutDistance targetDistance = Units.Meters.mutable(0);

        public Angle goalAngle() {
            return goalAngle;
        }

        public AngularVelocity goalVelocity() {
            return goalVelocity;
        }

        public Distance targetDistance() {
            return targetDistance;
        }
    }

    private final TurretSetpoint outputSetpoint = new TurretSetpoint();

    private Rotation2d lastGoalAngle = null;
    private LinearFilter turretAngleVelocityFilter = LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_PERIOD));

    private final StructPublisher<Translation2d> targetPublisher;
    private final StructPublisher<Pose2d> phaseDelayEstimatedPosePublisher;
    private final StructPublisher<Pose2d> lookaheadPosePublisher;
    private final StructPublisher<Pose2d> turretPositionPublisher;
    private final StructPublisher<Rotation2d> outputGoalAnglePublisher;
    private final DoublePublisher outputOmegaPublisher;
    private final DoublePublisher outputDistanceToTargetPublisher;

    public TurretTargeting(RobotPositioning positioning) {
        this.positioning = positioning;

        var table = NTHelpers.getTable("turret-targeting");
        targetPublisher = table.getStructTopic("target", Translation2d.struct).publish();
        phaseDelayEstimatedPosePublisher = table.getStructTopic("phase delay estimated pose", Pose2d.struct)
                .publish();
        lookaheadPosePublisher =
                table.getStructTopic("lookahead pose", Pose2d.struct).publish();
        turretPositionPublisher =
                table.getStructTopic("turret position", Pose2d.struct).publish();
        outputGoalAnglePublisher =
                table.getStructTopic("output goal angle", Rotation2d.struct).publish();
        outputOmegaPublisher =
                table.getDoubleTopic("output goal angular velocity").publish();
        outputDistanceToTargetPublisher =
                table.getDoubleTopic("output distance to target").publish();
    }

    /**
     * Get the estimated pose of the robot when the actual shooting happens.
     * <p>
     * Every system takes a fraction of a second between when a setpoint is commanded and the system achieves that
     * setpoint. We want to calculate our firing solution based on the position of the robot when the various
     * shooting systems are actually running. This is accounted for by an average phase delay.
     * <p>
     * See https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/282
     * @return the estimated position of the robot
     */
    private Pose2d getEstimatedPoseAfterPhaseDelay() {
        Pose2d robotPose = positioning.getRobotPose();
        ChassisSpeeds robotRelativeVelocity = positioning.getRobotVelocity();

        double phaseDelaySeconds = MoPrefs.turretPhaseDelay.get().in(Units.Seconds);
        var estimatedPose = robotPose.exp(new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelaySeconds,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelaySeconds,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelaySeconds));

        phaseDelayEstimatedPosePublisher.set(estimatedPose);
        return estimatedPose;
    }

    private double calculateTurretVelocityX(ChassisSpeeds robotVelocity, double robotAngle) {
        return robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (TurretSubsystem.robotToTurret.getY() * Math.cos(robotAngle)
                                - TurretSubsystem.robotToTurret.getX() * Math.sin(robotAngle));
    }

    private double calculateTurretVelocityY(ChassisSpeeds robotVelocity, double robotAngle) {
        return robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (TurretSubsystem.robotToTurret.getX() * Math.cos(robotAngle)
                                - TurretSubsystem.robotToTurret.getY() * Math.sin(robotAngle));
    }

    private final MutDistance distanceToTargetForTOFEstimation =
            HoodSerializedInformationHolder.DISTANCE_STORE_UNIT.mutable(0);

    private double estimateTimeOfFlight(double distanceToTarget) {
        return HoodSerializedInformationHolder.getInstance()
                .getTimeOfFlight(distanceToTargetForTOFEstimation.mut_replace(distanceToTarget, Units.Meters))
                .in(Units.Seconds);
    }

    /**
     * Account for the velocity applied to the fuel by the robot's movement by calculating the equivalent turret
     * pose as if the robot were not moving.
     */
    private Pose2d estimateLookaheadPose(
            Translation2d target, Pose2d turretPosition, ChassisSpeeds robotVelocity, Rotation2d robotAngle) {
        double turretVelocityX = calculateTurretVelocityX(robotVelocity, robotAngle.getRadians());
        double turretVelocityY = calculateTurretVelocityY(robotVelocity, robotAngle.getRadians());

        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = target.getDistance(turretPosition.getTranslation());
        for (int i = 0; i < NUM_LOOKAHEAD_ESTIMATION_ITERATIONS; i++) {
            timeOfFlight = estimateTimeOfFlight(lookaheadTurretToTargetDistance);
            double offsetX = turretVelocityX * timeOfFlight;
            double offsetY = turretVelocityY * timeOfFlight;
            lookaheadPose = new Pose2d(
                    turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                    turretPosition.getRotation());
            lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        lookaheadPosePublisher.set(lookaheadPose);
        return lookaheadPose;
    }

    private double calculateGoalVelocity(Rotation2d goalAngle) {
        if (lastGoalAngle == null) {
            lastGoalAngle = goalAngle;
        }

        double velocity = turretAngleVelocityFilter.calculate(
                goalAngle.minus(lastGoalAngle).getRadians() / Constants.LOOP_PERIOD);
        lastGoalAngle = goalAngle;
        return velocity;
    }

    private Pose2d getSOTMTurretPose(Translation2d target) {
        Pose2d estimatedPose = getEstimatedPoseAfterPhaseDelay();
        var turretPosition = estimatedPose.transformBy(TurretSubsystem.robotToTurret);
        turretPositionPublisher.set(turretPosition);

        ChassisSpeeds robotVelocity = positioning.getFieldVelocity();
        Rotation2d robotAngle = estimatedPose.getRotation();

        return estimateLookaheadPose(target, turretPosition, robotVelocity, robotAngle);
    }

    private Pose2d getStationaryTurretPose() {
        var robotPose = getEstimatedPoseAfterPhaseDelay();
        var turretPosition = robotPose.transformBy(TurretSubsystem.robotToTurret);
        turretPositionPublisher.set(turretPosition);

        return turretPosition;
    }

    private TurretSetpoint targetPosition(Translation2d target, Pose2d turretPose) {
        targetPublisher.set(target);

        Rotation2d goalAngle = target.minus(turretPose.getTranslation()).getAngle();
        double goalVelocity = calculateGoalVelocity(goalAngle);
        double turretToTargetDistance = target.getDistance(turretPose.getTranslation());

        Rotation2d robotRelativeGoalAngle =
                goalAngle.minus(positioning.getRobotPose().getRotation());
        double robotRelativeGoalVelocity = goalVelocity - positioning.getFieldVelocity().omegaRadiansPerSecond;

        outputSetpoint.goalAngle.mut_replace(robotRelativeGoalAngle.getRadians(), Units.Radians);
        outputSetpoint.goalVelocity.mut_replace(robotRelativeGoalVelocity, Units.RadiansPerSecond);
        outputSetpoint.targetDistance.mut_replace(turretToTargetDistance, Units.Meters);

        outputGoalAnglePublisher.set(robotRelativeGoalAngle);
        outputOmegaPublisher.set(robotRelativeGoalVelocity);
        outputDistanceToTargetPublisher.set(turretToTargetDistance);

        return outputSetpoint;
    }

    /**
     * Returns a targeting solution offset as required to account for the robot's velocity.
     */
    public TurretSetpoint targetPositionSOTM(Translation2d target) {
        return targetPosition(target, getSOTMTurretPose(target));
    }

    /**
     * Returns a direct targeting solution with no offset.
     */
    public TurretSetpoint targetPositionStationary(Translation2d target) {
        return targetPosition(target, getStationaryTurretPose());
    }
}
