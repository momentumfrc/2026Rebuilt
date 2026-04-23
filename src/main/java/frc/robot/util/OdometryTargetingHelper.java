package frc.robot.util;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;

public class OdometryTargetingHelper {
    public enum TargetType {
        HUB,
        SHUTTLE
    }

    private static final double FIELD_CENTER = inchesToMeters(158.84);

    private static final Translation3d BLUE_ALLIANCE_TARGET =
            new Translation3d(inchesToMeters(157.79 + 23.51), inchesToMeters(158.32), inchesToMeters(72.0));

    private static final Translation3d RED_ALLIANCE_TARGET =
            new Translation3d(inchesToMeters(492.33 - 23.51), inchesToMeters(158.32), inchesToMeters(72.0));

    private static final Translation3d BLUE_ALLIANCE_SHUTTLE_UPPER =
            new Translation3d(inchesToMeters(79.17), inchesToMeters(43.22), 0);
    private static final Translation3d BLUE_ALLIANCE_SHUTTLE_LOWER =
            new Translation3d(inchesToMeters(79.17), inchesToMeters(274.47), 0);
    private static final Translation3d RED_ALLIANCE_SHUTTLE_UPPER =
            new Translation3d(inchesToMeters(571.89), inchesToMeters(43.22), 0);
    private static final Translation3d RED_ALLIANCE_SHUTTLE_LOWER =
            new Translation3d(inchesToMeters(571.89), inchesToMeters(274.47), 0);

    public static Translation3d getHubTarget(DriverStation.Alliance alliance) {
        return alliance == DriverStation.Alliance.Blue ? BLUE_ALLIANCE_TARGET : RED_ALLIANCE_TARGET;
    }

    public static Translation3d getShuttleTarget(DriverStation.Alliance alliance, Pose2d currPos) {
        double currY = currPos.getY();
        if (currY > FIELD_CENTER) {
            return alliance == DriverStation.Alliance.Blue ? BLUE_ALLIANCE_SHUTTLE_LOWER : RED_ALLIANCE_SHUTTLE_LOWER;
        } else {
            return alliance == DriverStation.Alliance.Blue ? BLUE_ALLIANCE_SHUTTLE_UPPER : RED_ALLIANCE_SHUTTLE_UPPER;
        }
    }

    public static Translation3d getTarget(DriverStation.Alliance alliance, Pose2d currPos, TargetType targetType) {
        return switch (targetType) {
            case HUB -> getHubTarget(alliance);
            case SHUTTLE -> getShuttleTarget(alliance, currPos);
        };
    }

    private OdometryTargetingHelper() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [OdometryTargetingHelper]");
    }
}
