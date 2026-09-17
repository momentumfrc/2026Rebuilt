package first.robot.util;

import org.wpilib.math.interpolation.Interpolator;
import org.wpilib.math.interpolation.InverseInterpolator;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;

public class TurretAngleHelper {
    private static final double FLOAT_DELTA = 1e-9;

    private final Angle minAngle;
    private final Angle maxAngle;

    public record Result(Angle angle, boolean inRange) {}

    public TurretAngleHelper(Angle minAngle, Angle maxAngle) {
        if (maxAngle.baseUnitMagnitude() <= minAngle.baseUnitMagnitude()) {
            throw new IllegalArgumentException("max angle must be greater than min angle");
        }
        if (maxAngle.minus(minAngle).in(Units.Radians) > 2 * Math.PI) {
            throw new IllegalArgumentException("this class only supports range < 360°");
        }
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    public Result turretAngleModulus(Angle angle) {
        return turretAngleModulusRads(angle.in(Units.Radians));
    }

    public Result turretAngleModulusRads(double rads) {
        double minRad = minAngle.in(Units.Radians);
        double maxRad = maxAngle.in(Units.Radians);

        double value = MathUtil.inputModulus(rads - minRad, 0, 2 * Math.PI);
        if (value == 2 * Math.PI) {
            value = 0;
        }

        if (value <= maxRad - minRad) {
            return new Result(Units.Radians.of(minRad + value), true);
        } else {
            // might want to check this
            value = Interpolator.forDouble()
                    .interpolate(
                            maxRad - minRad,
                            0d,
                            InverseInterpolator.forDouble().inverseInterpolate(maxRad - minRad, 2 * Math.PI, value));
            return new Result(Units.Radians.of(minRad + value), false);
        }
    }
}
