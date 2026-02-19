package frc.robot.shootutils;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Map;
import org.junit.jupiter.api.Test;

public class HoodSerializedInformationHolderTests {
    private static final double TOLERANCE = 1e-6;

    private HoodSerializedInformationHolder fromAngleEntries(Map<Double, Double> angleMap) {
        var entries = angleMap.entrySet().stream()
                .map(entry -> new HoodSerializedInformationHolder.Entry(entry.getKey(), entry.getValue(), null, null))
                .toList();
        return new HoodSerializedInformationHolder(entries);
    }

    private HoodSerializedInformationHolder fromSpeedEntries(Map<Double, Double> speedsMap) {
        var entries = speedsMap.entrySet().stream()
                .map(entry -> new HoodSerializedInformationHolder.Entry(entry.getKey(), null, entry.getValue(), null))
                .toList();
        return new HoodSerializedInformationHolder(entries);
    }

    @Test
    public void testAngleInterpolation() {
        HoodSerializedInformationHolder holder = fromAngleEntries(Map.of(1.0, 5.0, 2.0, 6.0));

        assertEquals(5.0, holder.getHoodAngle(1.0), TOLERANCE);
        assertEquals(6.0, holder.getHoodAngle(2.0), TOLERANCE);

        assertEquals(5.5, holder.getHoodAngle(1.5), TOLERANCE);
        assertEquals(5.25, holder.getHoodAngle(1.25), TOLERANCE);

        assertEquals(5.0, holder.getHoodAngle(0), TOLERANCE);
        assertEquals(5.0, holder.getHoodAngle(0.5), TOLERANCE);

        assertEquals(6.0, holder.getHoodAngle(2.5), TOLERANCE);
        assertEquals(6.0, holder.getHoodAngle(4), TOLERANCE);
    }

    @Test
    public void testFlywheelSpeedInterpolation() {
        HoodSerializedInformationHolder holder = fromSpeedEntries(Map.of(5.0, 1500.0, 10.0, 1600.0));

        assertEquals(1500.0, holder.getFlywheelSpeed(5.0), TOLERANCE);
        assertEquals(1600.0, holder.getFlywheelSpeed(10.0), TOLERANCE);

        assertEquals(1520.0, holder.getFlywheelSpeed(6.0), TOLERANCE);
        assertEquals(1550.0, holder.getFlywheelSpeed(7.5), TOLERANCE);

        assertEquals(1500.0, holder.getFlywheelSpeed(4.0), TOLERANCE);
        assertEquals(1600.0, holder.getFlywheelSpeed(11.0), TOLERANCE);
    }

    @Test
    public void testAngleInterpolationNonLinear() {
        HoodSerializedInformationHolder holder = fromAngleEntries(Map.of(1.0, 5.0, 2.0, 6.0, 3.5, 9.0));

        assertEquals(5.0, holder.getHoodAngle(0.5), TOLERANCE);
        assertEquals(5.5, holder.getHoodAngle(1.5), TOLERANCE);
        assertEquals(7.0, holder.getHoodAngle(2.5), TOLERANCE);
        assertEquals(9.0, holder.getHoodAngle(4.0), TOLERANCE);
    }

    @Test
    public void testFlywheelSpeedInterpolationNonLinear() {
        HoodSerializedInformationHolder holder = fromSpeedEntries(Map.of(5.0, 1500.0, 10.0, 1600.0, 20.0, 1700.0));

        assertEquals(1500.0, holder.getFlywheelSpeed(4.0), TOLERANCE);
        assertEquals(1550.0, holder.getFlywheelSpeed(7.5), TOLERANCE);
        assertEquals(1610.0, holder.getFlywheelSpeed(11.0), TOLERANCE);
        assertEquals(1650.0, holder.getFlywheelSpeed(15.0), TOLERANCE);
        assertEquals(1700.0, holder.getFlywheelSpeed(21.0), TOLERANCE);
    }
}
