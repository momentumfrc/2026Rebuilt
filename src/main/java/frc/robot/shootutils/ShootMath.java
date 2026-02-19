package frc.robot.shootutils;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutDistance;
import java.util.List;
import java.util.function.Supplier;

public class ShootMath {
    public static class Solution {
        private final MutDistance distance = Units.Meters.mutable(0);
        private final MutAngle hoodAngle = Units.Radians.mutable(0);
        private final MutAngularVelocity flywheelSpeed = Units.RPM.mutable(0);

        public Distance distance() {
            return distance;
        }

        public Angle hoodAngle() {
            return hoodAngle;
        }

        public AngularVelocity flywheelSpeed() {
            return flywheelSpeed;
        }
    }

    private static void interpolate(
            List<ShooterEmpiricalDataStore.Datum> data,
            Distance distance,
            MutAngle hoodAngle,
            MutAngularVelocity flywheelSpeed) {
        assert data.size() >= 2;

        int lowerBound = 0;
        int upperBound = data.size() - 1;
        if (distance.lt(data.get(lowerBound).distance())) {
            upperBound = 1;
        } else if (distance.gt(data.get(upperBound).distance())) {
            lowerBound = data.size() - 2;
        } else {
            for (int i = 1; i < data.size(); i++) {
                if (distance.lt(data.get(i).distance())) {
                    lowerBound = i - 1;
                    upperBound = i;
                    break;
                }
            }
        }
        var lowerDatum = data.get(lowerBound);
        var upperDatum = data.get(upperBound);

        assert lowerDatum.distance().lt(upperDatum.distance());

        double run =
                upperDatum.distance().in(Units.Meters) - lowerDatum.distance().in(Units.Meters);

        double angleM = (upperDatum.hoodAngle().in(Units.Radians)
                        - lowerDatum.hoodAngle().in(Units.Radians))
                / run;

        double speedM = (upperDatum.flywheelSpeed().in(Units.RPM)
                        - lowerDatum.flywheelSpeed().in(Units.RPM))
                / run;

        double distDiff = distance.in(Units.Meters) - lowerDatum.distance().in(Units.Meters);

        hoodAngle.mut_replace(lowerDatum.hoodAngle().in(Units.Radians) + (angleM * distDiff), Units.Radians);

        flywheelSpeed.mut_replace(lowerDatum.flywheelSpeed().in(Units.RPM) + (speedM * distDiff), Units.RPM);
    }

    static Supplier<List<ShooterEmpiricalDataStore.Datum>> dataSupplier =
            () -> ShooterEmpiricalDataStore.getInstance().getSortedData();

    private static Solution solution = new Solution();

    public static Solution solve(Distance distance) {
        if (solution.distance().isNear(distance, 1e-6)) {
            return solution;
        }

        var data = dataSupplier.get();

        solution.distance.mut_replace(distance);
        interpolate(data, distance, solution.hoodAngle, solution.flywheelSpeed);

        return solution;
    }
}
