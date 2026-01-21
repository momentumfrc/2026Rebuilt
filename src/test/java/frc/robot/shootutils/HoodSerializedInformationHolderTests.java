package frc.robot.shootutils;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Collections;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class HoodSerializedInformationHolderTests {
    private static final double TOLERANCE = 1e-6;

    @Test
    public void testAngleInterpolation() {
        HoodSerializedInformationHolder holder =
                new HoodSerializedInformationHolder(Map.of(1.0, 5.0, 2.0, 6.0), Collections.emptyMap());

        assertEquals(5.0, holder.getAngle(1.0), TOLERANCE);
        assertEquals(6.0, holder.getAngle(2.0), TOLERANCE);

        assertEquals(5.5, holder.getAngle(1.5), TOLERANCE);
        assertEquals(5.25, holder.getAngle(1.25), TOLERANCE);

        assertEquals(4, holder.getAngle(0), TOLERANCE);
        assertEquals(4.5, holder.getAngle(0.5), TOLERANCE);

        assertEquals(6.5, holder.getAngle(2.5), TOLERANCE);
        assertEquals(8.0, holder.getAngle(4), TOLERANCE);
    }

    @Test
    public void testFlywheelSpeedInterpolation() {
        HoodSerializedInformationHolder holder =
                new HoodSerializedInformationHolder(Collections.emptyMap(), Map.of(5.0, 1500.0, 10.0, 1600.0));

        assertEquals(1500.0, holder.getFlywheelSpeed(5.0), TOLERANCE);
        assertEquals(1600.0, holder.getFlywheelSpeed(10.0), TOLERANCE);

        assertEquals(1520.0, holder.getFlywheelSpeed(6.0), TOLERANCE);
        assertEquals(1550.0, holder.getFlywheelSpeed(7.5), TOLERANCE);

        assertEquals(1480.0, holder.getFlywheelSpeed(4.0), TOLERANCE);
        assertEquals(1620.0, holder.getFlywheelSpeed(11.0), TOLERANCE);
    }

    @Test
    public void testAngleInterpolationNonLinear() {
        HoodSerializedInformationHolder holder =
                new HoodSerializedInformationHolder(Map.of(1.0, 5.0, 2.0, 6.0, 3.5, 9.0), Collections.emptyMap());

        assertEquals(4.5, holder.getAngle(0.5), TOLERANCE);
        assertEquals(5.5, holder.getAngle(1.5), TOLERANCE);
        assertEquals(7.0, holder.getAngle(2.5), TOLERANCE);
        assertEquals(10.0, holder.getAngle(4.0), TOLERANCE);
    }

    @Test
    public void testFlywheelSpeedInterpolationNonLinear() {
        HoodSerializedInformationHolder holder = new HoodSerializedInformationHolder(
                Collections.emptyMap(), Map.of(5.0, 1500.0, 10.0, 1600.0, 20.0, 1700.0));

        assertEquals(1480.0, holder.getFlywheelSpeed(4.0), TOLERANCE);
        assertEquals(1550.0, holder.getFlywheelSpeed(7.5), TOLERANCE);
        assertEquals(1610.0, holder.getFlywheelSpeed(11.0), TOLERANCE);
        assertEquals(1650.0, holder.getFlywheelSpeed(15.0), TOLERANCE);
        assertEquals(1710.0, holder.getFlywheelSpeed(21.0), TOLERANCE);
    }
}
