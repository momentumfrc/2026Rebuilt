package frc.robot.shootutils;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.units.Units;
import frc.robot.shootutils.ShooterEmpiricalDataStore.Datum;
import java.util.Comparator;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

public class ShootMathTests {
    private static final double TOLERANCE = 1e-6;

    private static List<Datum> testData;

    @BeforeAll
    public static void setupDataSupplier() {
        ShootMath.dataSupplier = () -> testData;
    }

    private List<Datum> dataFromAngleMap(Map<Double, Double> distToAngleMap) {
        return distToAngleMap.entrySet().stream()
                .map(entry -> new Datum(
                        Units.Meters.of(entry.getKey()), Units.Degrees.of(entry.getValue()), Units.RPM.zero()))
                .sorted(Comparator.comparing(d -> d.distance()))
                .toList();
    }

    private List<Datum> dataFromSpeedMap(Map<Double, Double> distToSpeedMap) {
        return distToSpeedMap.entrySet().stream()
                .map(entry -> new Datum(
                        Units.Meters.of(entry.getKey()), Units.Degrees.zero(), Units.RPM.of(entry.getValue())))
                .sorted(Comparator.comparing(d -> d.distance()))
                .toList();
    }

    double getAngle(double distanceMeters) {
        return ShootMath.solve(Units.Meters.of(distanceMeters)).hoodAngle().in(Units.Degrees);
    }

    double getFlywheelSpeed(double distanceMeters) {
        return ShootMath.solve(Units.Meters.of(distanceMeters)).flywheelSpeed().in(Units.RPM);
    }

    @Test
    public void testAngleInterpolation() {
        testData = dataFromAngleMap(Map.of(1.0, 5.0, 2.0, 6.0));

        assertEquals(5.0, getAngle(1.0), TOLERANCE);
        assertEquals(6.0, getAngle(2.0), TOLERANCE);

        assertEquals(5.5, getAngle(1.5), TOLERANCE);
        assertEquals(5.25, getAngle(1.25), TOLERANCE);

        assertEquals(4, getAngle(0), TOLERANCE);
        assertEquals(4.5, getAngle(0.5), TOLERANCE);

        assertEquals(6.5, getAngle(2.5), TOLERANCE);
        assertEquals(8.0, getAngle(4), TOLERANCE);
    }

    @Test
    public void testFlywheelSpeedInterpolation() {
        testData = dataFromSpeedMap(Map.of(5.0, 1500.0, 10.0, 1600.0));

        assertEquals(1500.0, getFlywheelSpeed(5.0), TOLERANCE);
        assertEquals(1600.0, getFlywheelSpeed(10.0), TOLERANCE);

        assertEquals(1520.0, getFlywheelSpeed(6.0), TOLERANCE);
        assertEquals(1550.0, getFlywheelSpeed(7.5), TOLERANCE);

        assertEquals(1480.0, getFlywheelSpeed(4.0), TOLERANCE);
        assertEquals(1620.0, getFlywheelSpeed(11.0), TOLERANCE);
    }

    @Test
    public void testAngleInterpolationNonLinear() {
        testData = dataFromAngleMap(Map.of(1.0, 5.0, 2.0, 6.0, 3.5, 9.0));

        assertEquals(4.5, getAngle(0.5), TOLERANCE);
        assertEquals(5.5, getAngle(1.5), TOLERANCE);
        assertEquals(7.0, getAngle(2.5), TOLERANCE);
        assertEquals(10.0, getAngle(4.0), TOLERANCE);
    }

    @Test
    public void testFlywheelSpeedInterpolationNonLinear() {
        testData = dataFromSpeedMap(Map.of(5.0, 1500.0, 10.0, 1600.0, 20.0, 1700.0));

        assertEquals(1480.0, getFlywheelSpeed(4.0), TOLERANCE);
        assertEquals(1550.0, getFlywheelSpeed(7.5), TOLERANCE);
        assertEquals(1610.0, getFlywheelSpeed(11.0), TOLERANCE);
        assertEquals(1650.0, getFlywheelSpeed(15.0), TOLERANCE);
        assertEquals(1710.0, getFlywheelSpeed(21.0), TOLERANCE);
    }
}
