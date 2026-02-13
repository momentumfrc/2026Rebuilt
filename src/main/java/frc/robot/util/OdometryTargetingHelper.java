package frc.robot.util;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class OdometryTargetingHelper {
    private static final Translation3d BLUE_ALLIANCE_TARGET =
            new Translation3d(inchesToMeters(157.79 + 23.51), inchesToMeters(158.32), inchesToMeters(72.0));

    private static final Translation3d RED_ALLIANCE_TARGET =
            new Translation3d(inchesToMeters(492.33 - 23.51), inchesToMeters(158.32), inchesToMeters(72.0));

    public static Translation3d getTarget(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? BLUE_ALLIANCE_TARGET : RED_ALLIANCE_TARGET;
    }

    /**
     * Gets the translation from the robot to the target.
     * <p>
     * The angle towards the target and distance to the target can be determined using the getAngle() and getNorm()
     * methods on the returned transform.
     * @param robotPose current robot pose, in field coordinates, with the blue alliance origin.
     */
    public static Translation2d getTranslationToTarget(Translation2d robotPose, DriverStation.Alliance alliance) {
        return getTarget(alliance).toTranslation2d().minus(robotPose);
    }

    private OdometryTargetingHelper() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [OdometryTargetingHelper]");
    }
}
