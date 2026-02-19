package frc.robot.shootutils;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.RobotPositioning;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.NTHelpers;

/**
 * Logic to perform turret targeting for shoot-on-the-fly.
 * <p>
 * Heavily based on https://www.chiefdelphi.com/t/frc-6328-mechanical-advantage-2026-build-thread/509595/272
 */
public final class TurretTargeting {
    private static final int NUM_LOOKAHEAD_ESTIMATION_ITERATIONS = 20;

    private static final Transform2d robotToTurret = new Transform2d(
            TurretSubsystem.robotToTurret.getTranslation().toTranslation2d(),
            TurretSubsystem.robotToTurret.getRotation().toRotation2d());

    public enum TurretTargetMode {
        SOTM,
        STATIONARY
    }

    private final SendableChooser<TurretTargeting.TurretTargetMode> targetModeChooser =
            NTHelpers.enumToChooser(TurretTargeting.TurretTargetMode.class);

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

    private final StructLogEntry<Translation2d> targetLogger;
    private final StructLogEntry<Pose2d> phaseDelayEstimatedPoseLogger;
    private final StructLogEntry<Pose2d> lookaheadPoseLogger;
    private final StructLogEntry<Pose2d> turretPositionLogger;
    private final StructLogEntry<Rotation2d> outputGoalAngleLogger;
    private final DoubleLogEntry outputOmegaLogger;
    private final DoubleLogEntry outputDistanceToTargetLogger;
    private final StringLogEntry targetingModeLogger;

    public TurretTargeting(RobotPositioning positioning) {
        this.positioning = positioning;

        var log = DataLogManager.getLog();
        final String logPrefix = "sotf-targeting/";
        targetLogger = StructLogEntry.create(log, logPrefix + "target", Translation2d.struct);
        phaseDelayEstimatedPoseLogger =
                StructLogEntry.create(log, logPrefix + "phase delay estimated pose", Pose2d.struct);
        lookaheadPoseLogger = StructLogEntry.create(log, logPrefix + "lookahead pose", Pose2d.struct);
        turretPositionLogger = StructLogEntry.create(log, logPrefix + "turret position", Pose2d.struct);
        outputGoalAngleLogger = StructLogEntry.create(log, logPrefix + "output goal angle", Rotation2d.struct);
        outputOmegaLogger = new DoubleLogEntry(log, logPrefix + "output goal angular velocity");
        outputDistanceToTargetLogger = new DoubleLogEntry(log, logPrefix + "output distance to target");
        targetingModeLogger = new StringLogEntry(log, logPrefix + "targeting mode");
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
        return robotPose.exp(new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelaySeconds,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelaySeconds,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelaySeconds));
    }

    private double calculateTurretVelocityX(ChassisSpeeds robotVelocity, double robotAngle) {
        return robotVelocity.vxMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (robotToTurret.getY() * Math.cos(robotAngle) - robotToTurret.getX() * Math.sin(robotAngle));
    }

    private double calculateTurretVelocityY(ChassisSpeeds robotVelocity, double robotAngle) {
        return robotVelocity.vyMetersPerSecond
                + robotVelocity.omegaRadiansPerSecond
                        * (robotToTurret.getX() * Math.cos(robotAngle) - robotToTurret.getY() * Math.sin(robotAngle));
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

        return lookaheadPose;
    }

    private double calculateGoalVelocity(Rotation2d goalAngle) {
        if (lastGoalAngle == null) {
            lastGoalAngle = goalAngle;
        }

        return turretAngleVelocityFilter.calculate(
                goalAngle.minus(lastGoalAngle).getRadians() / Constants.LOOP_PERIOD);
    }

    private Pose2d getSOTMTurretPose(Translation2d target) {
        Pose2d estimatedPose = getEstimatedPoseAfterPhaseDelay();
        phaseDelayEstimatedPoseLogger.append(estimatedPose);

        var turretPosition = estimatedPose.transformBy(robotToTurret);
        turretPositionLogger.append(turretPosition);

        ChassisSpeeds robotVelocity = positioning.getFieldVelocity();
        Rotation2d robotAngle = estimatedPose.getRotation();

        Pose2d lookaheadPose = estimateLookaheadPose(target, turretPosition, robotVelocity, robotAngle);
        lookaheadPoseLogger.append(lookaheadPose);

        return lookaheadPose;
    }

    private Pose2d getStationaryTurretPose() {
        var robotPose = positioning.getRobotPose();
        var turretPosition = robotPose.transformBy(robotToTurret);
        turretPositionLogger.append(turretPosition);
        return turretPosition;
    }

    public TurretSetpoint targetPosition(Translation2d target, TurretTargetMode targetMode) {
        targetLogger.append(target);
        targetingModeLogger.append(targetMode.toString());

        Pose2d turretPose =
                switch (targetMode) {
                    case SOTM -> getSOTMTurretPose(target);
                    case STATIONARY -> getStationaryTurretPose();
                };

        Rotation2d goalAngle = target.minus(turretPose.getTranslation()).getAngle();
        double goalVelocity = calculateGoalVelocity(goalAngle);
        double turretToTargetDistance = target.getDistance(turretPose.getTranslation());

        Rotation2d robotRelativeGoalAngle =
                goalAngle.minus(positioning.getRobotPose().getRotation());
        double robotRelativeGoalVelocity = goalVelocity - positioning.getFieldVelocity().omegaRadiansPerSecond;

        outputSetpoint.goalAngle.mut_replace(robotRelativeGoalAngle.getRadians(), Units.Radians);
        outputSetpoint.goalVelocity.mut_replace(robotRelativeGoalVelocity, Units.RadiansPerSecond);
        outputSetpoint.targetDistance.mut_replace(turretToTargetDistance, Units.Meters);

        outputGoalAngleLogger.append(robotRelativeGoalAngle);
        outputOmegaLogger.append(robotRelativeGoalVelocity);
        outputDistanceToTargetLogger.append(turretToTargetDistance);

        return outputSetpoint;
    }

    public TurretSetpoint targetPosition(Translation2d target) {
        return targetPosition(target, targetModeChooser.getSelected());
    }
}
