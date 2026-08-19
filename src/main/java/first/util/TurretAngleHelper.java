package first.util;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.interpolation.Interpolator;
import org.wpilib.math.interpolation.InverseInterpolator;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;

public class TurretAngleHelper {
    private static final double FLOAT_DELTA = 1e-9;

    private final Rotation2d minAngle;
    private final Rotation2d maxAngle;

    private MutAngle mutAngle = Units.Radians.mutable(0);

    public static class Result {
        private Rotation2d angle;
        private boolean inRange;

        public Rotation2d angle() {
            return angle;
        }

        public boolean inRange() {
            return inRange;
        }
    }

    private Result result = new Result();

    public TurretAngleHelper(Rotation2d minAngle, Rotation2d maxAngle) {
        if (maxAngle.getRadians() <= minAngle.getRadians()) {
            throw new IllegalArgumentException("max angle must be greater than min angle");
        }
        if (maxAngle.minus(minAngle).getRadians() >= 2 * Math.PI) {
            throw new IllegalArgumentException("this class only supports range < 360°");
        }

        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    /**
     * Limits values to within [minAngle, maxAngle]. Returns null if a value is out of range.
     */
    public Result turretAngleModulus(Rotation2d angle) {
        return turretAngleModulusRads(angle.getRadians());
    }

    public Result turretAngleModulus(Angle angle) {
        return turretAngleModulusRads(angle.in(Units.Radians));
    }

    public Result turretAngleModulusRads(double rads) {
        double minRad = minAngle.getRadians();
        double maxRad = maxAngle.getRadians();

        double value = MathUtil.inputModulus(rads - minRad, 0, 2 * Math.PI);
        if (value == 2 * Math.PI) {
            value = 0;
        }

        if (value <= maxRad - minRad) {
            result.inRange = true;
            result.angle = Rotation2d.fromRadians(minRad + value);
        } else {
            result.inRange = false;
            //might want to check this
            value = Interpolator.forDouble().interpolate(
                    maxRad - minRad, 0d, InverseInterpolator.forDouble().inverseInterpolate(maxRad - minRad, 2 * Math.PI, value));
            result.angle = Rotation2d.fromRadians(minRad + value);
        }

        return result;
    }
}
