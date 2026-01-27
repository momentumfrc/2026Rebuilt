package frc.robot.shootutils;

import static org.junit.jupiter.api.Assertions.*;

import java.util.Collections;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

public class HoodSerializedInformationHolderTests {
    private static final double TOLERANCE = 1e-6;

    @Test
    public void testAngleInterpolation() {
        HoodSerializedInformationHolder holder =
                new HoodSerializedInformationHolder(List.of(new HoodSerializedInformationHolder.Entry(1f, 1500f, 5f), new HoodSerializedInformationHolder.Entry(2f, 1600f, 6f)));

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
                new HoodSerializedInformationHolder(List.of(new HoodSerializedInformationHolder.Entry(5f, 1500f, 5f), new HoodSerializedInformationHolder.Entry(10f, 1600f, 6f)));

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
                new HoodSerializedInformationHolder(List.of(new HoodSerializedInformationHolder.Entry(1f, 1500f, 5f), new HoodSerializedInformationHolder.Entry(2f, 1600f, 6f), new HoodSerializedInformationHolder.Entry(3.5f, 1700f, 9f)));

        assertEquals(4.5, holder.getAngle(0.5), TOLERANCE);
        assertEquals(5.5, holder.getAngle(1.5), TOLERANCE);
        assertEquals(7.0, holder.getAngle(2.5), TOLERANCE);
        assertEquals(10.0, holder.getAngle(4.0), TOLERANCE);
    }

    @Test
    public void testFlywheelSpeedInterpolationNonLinear() {
        HoodSerializedInformationHolder holder =
                new HoodSerializedInformationHolder(List.of(new HoodSerializedInformationHolder.Entry(5f, 1500f, 5f), new HoodSerializedInformationHolder.Entry(10f, 1600f, 6f), new HoodSerializedInformationHolder.Entry(20f, 1700f, 7f)));

        assertEquals(1480.0, holder.getFlywheelSpeed(4.0), TOLERANCE);
        assertEquals(1550.0, holder.getFlywheelSpeed(7.5), TOLERANCE);
        assertEquals(1610.0, holder.getFlywheelSpeed(11.0), TOLERANCE);
        assertEquals(1650.0, holder.getFlywheelSpeed(15.0), TOLERANCE);
        assertEquals(1710.0, holder.getFlywheelSpeed(21.0), TOLERANCE);
    }
}
