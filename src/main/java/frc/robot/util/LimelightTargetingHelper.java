package frc.robot.util;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.LimelightHelpers.RawFiducial;
import java.util.Map;

public class LimelightTargetingHelper {
    /**
     * If we always target the nearest fiducial, we could get into a state where 2 fiducials are near-equidistant,
     * causing the logic to flip-flop between them (due to noise), leading to degraded tracking performance.
     * <p>
     * To solve this, the algorithm will only swap to a nearer fiducial if the currently tracked fiducial is
     * at least SWAP_TARGETING_FIDUCIAL_CUTOFF meters further away.
     */
    private static final double SWAP_TARGETING_FIDUCIAL_CUTOFF = 0.1;

    private static final Map<Integer, Translation3d> redAllianceTags = Map.of(
            8, new Translation3d(-Units.inchesToMeters(23.50), -Units.inchesToMeters(14), Units.inchesToMeters(27.75)),
            5, new Translation3d(-Units.inchesToMeters(23.50), 0, Units.inchesToMeters(27.75)),
            9, new Translation3d(-Units.inchesToMeters(23.51), Units.inchesToMeters(14), Units.inchesToMeters(27.75)),
            10, new Translation3d(-Units.inchesToMeters(23.51), 0, Units.inchesToMeters(27.75)),
            11, new Translation3d(-Units.inchesToMeters(23.50), Units.inchesToMeters(14), Units.inchesToMeters(27.75)),
            2, new Translation3d(-Units.inchesToMeters(23.50), 0, Units.inchesToMeters(27.75)));
    private static final Map<Integer, Translation3d> blueAllianceTags = Map.of(
            21, new Translation3d(-Units.inchesToMeters(23.50), 0, Units.inchesToMeters(27.75)),
            24, new Translation3d(-Units.inchesToMeters(23.50), -Units.inchesToMeters(14), Units.inchesToMeters(27.75)),
            25, new Translation3d(-Units.inchesToMeters(23.51), Units.inchesToMeters(14), Units.inchesToMeters(27.75)),
            26, new Translation3d(-Units.inchesToMeters(23.51), 0, Units.inchesToMeters(27.75)),
            27, new Translation3d(-Units.inchesToMeters(23.50), Units.inchesToMeters(14), Units.inchesToMeters(27.75)),
            18, new Translation3d(-Units.inchesToMeters(23.50), 0, Units.inchesToMeters(27.75)));

    private final String limelightName;
    private int lastConfiguredTag = -1;

    public LimelightTargetingHelper(String limelightName) {
        this.limelightName = limelightName;
    }

    /**
     * Setup the limelight to target the nearest visible configured apriltag for the given alliance.
     */
    public void targetNearestTag(DriverStation.Alliance alliance) {
        var fiducials = LimelightHelpers.getRawFiducials(limelightName);
        if (fiducials.length == 0) {
            return;
        }

        var tagSet = alliance == DriverStation.Alliance.Blue ? blueAllianceTags : redAllianceTags;

        RawFiducial currFiducial = null;
        RawFiducial nearestFiducial = null;
        for (RawFiducial fiducial : fiducials) {
            if (tagSet.containsKey(fiducial.id) == false) {
                continue;
            }

            if (fiducial.id == lastConfiguredTag) {
                currFiducial = fiducial;
            }

            if (nearestFiducial == null || fiducial.distToCamera < nearestFiducial.distToCamera) {
                nearestFiducial = fiducial;
            }
        }

        if (nearestFiducial == null) {
            return;
        }

        if (currFiducial != null) {
            if (currFiducial.id == nearestFiducial.id) {
                return;
            }
            if (currFiducial.distToCamera - nearestFiducial.distToCamera < SWAP_TARGETING_FIDUCIAL_CUTOFF) {
                return;
            }
        }

        lastConfiguredTag = nearestFiducial.id;
        var fiducialOffset = tagSet.get(nearestFiducial.id);
        LimelightHelpers.setPriorityTagID(limelightName, nearestFiducial.id);
        LimelightHelpers.setFiducial3DOffset(
                limelightName, fiducialOffset.getX(), fiducialOffset.getY(), fiducialOffset.getZ());
    }

    public boolean targetIsVisible() {
        if (lastConfiguredTag < 0) {
            return false;
        }
        return LimelightHelpers.getTV(limelightName);
    }

    public int getTargetId() {
        return lastConfiguredTag;
    }

    public double getTx() {
        return LimelightHelpers.getTX(limelightName);
    }
}
