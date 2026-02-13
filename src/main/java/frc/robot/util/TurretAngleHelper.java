package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;

public class TurretAngleHelper {
    private static final double FLOAT_DELTA = 1e-9;

    private final Rotation2d minAngle;
    private final Rotation2d maxAngle;

    private MutAngle mutAngle = Units.Radians.mutable(0);

    public TurretAngleHelper(Rotation2d minAngle, Rotation2d maxAngle) {
        if (maxAngle.getRadians() <= minAngle.getRadians()) {
            throw new IllegalArgumentException("max angle must be greater than min angle");
        }

        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    /**
     * Limits values to within [minAngle, maxAngle]. Returns null if a value is out of range.
     */
    public Rotation2d turretAngleModulus(Rotation2d angle) {
        return Rotation2d.fromRadians(turretAngleModulusRads(angle.getRadians()));
    }

    public Angle turretAngleModulus(Angle angle) {
        return mutAngle.mut_replace(turretAngleModulusRads(angle.in(Units.Radians)), Units.Radians);
    }

    private double turretAngleModulusRads(double rads) {
        double min = minAngle.getRadians();
        double max = maxAngle.getRadians();

        if (rads < min) {
            rads = MathUtil.inputModulus(rads, min, min + 2 * Math.PI);
        }
        if (rads > max) {
            rads = MathUtil.inputModulus(rads, max - 2 * Math.PI, max);
        }

        if (min - rads > FLOAT_DELTA || rads - max > FLOAT_DELTA) {
            return Double.NaN;
        }

        return rads;
    }
}
