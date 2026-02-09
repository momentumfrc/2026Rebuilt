package frc.robot.util;

import static edu.wpi.first.math.util.Units.inchesToMeters;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DriverStation;

public class OdometryTargetingHelper {
    private static final Translation3d BLUE_ALLIANCE_TARGET =
            new Translation3d(inchesToMeters(157.79 + 23.51), inchesToMeters(158.32), inchesToMeters(72.0));

    private static final Translation3d RED_ALLIANCE_TARGET =
            new Translation3d(inchesToMeters(492.88 - 23.51), inchesToMeters(158.32), inchesToMeters(72));

    private static MutAngle targetingAngle = Units.Rotations.mutable(0);

    /**
     * Gets the angle towards the target, relative to the front of the robot.
     * @param robotPose current robot pose, with the blue alliance origin.
     */
    public static Angle getTargetAngle(Pose3d robotPose, DriverStation.Alliance alliance) {
        Translation3d target = alliance == DriverStation.Alliance.Blue ? BLUE_ALLIANCE_TARGET : RED_ALLIANCE_TARGET;
        // TODO: math
        return targetingAngle.mut_replace(0, Units.Radians);
    }

    private OdometryTargetingHelper() {
        throw new UnsupportedOperationException("Cannot instantiate static utility class [OdometryTargetingHelper]");
    }
}
