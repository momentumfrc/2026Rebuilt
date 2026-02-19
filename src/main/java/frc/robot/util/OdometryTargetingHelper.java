package frc.robot.util;

import static edu.wpi.first.math.util.Units.inchesToMeters;

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

    private OdometryTargetingHelper() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [OdometryTargetingHelper]");
    }
}
