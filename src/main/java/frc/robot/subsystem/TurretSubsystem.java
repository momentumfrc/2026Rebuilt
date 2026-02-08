package frc.robot.subsystem;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TurretSubsystem extends SubsystemBase {

    // TODO: fill this out once the robot is designed
    private static final Translation3d turretPositionInRobotCoordinates =
            new Translation3d(-0.154305, -0.031750, 0.381);
    private static final Transform3d limelightPositionRelativeToTurret =
            new Transform3d(0.181656, 0, 0.146352, new Rotation3d(0, degreesToRadians(15), 0));

    private MutAngle mutTurretPosition = Units.Rotations.mutable(0);
    /**
     * Get the current angular position of the turret about its axis of rotation.
     */
    public Angle getTurretYaw() {
        // TODO: update the mutable measure with the actual position
        return mutTurretPosition;
    }

    /**
     * Get the current pose of the turret in robot-relative coordinates.
     */
    public Pose3d getTurretPose() {
        return new Pose3d(
                turretPositionInRobotCoordinates,
                new Rotation3d(0, 0, getTurretYaw().in(Units.Radians)));
    }

    /**
     * Get the current pose of the targeting limelight attached to the turret in robot-relative coordinates.
     */
    public Pose3d getTurretLimelightPose() {
        return getTurretPose().plus(limelightPositionRelativeToTurret);
    }
}
