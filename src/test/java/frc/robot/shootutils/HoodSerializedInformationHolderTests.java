package frc.robot.shootutils;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

public class HoodSerializedInformationHolderTests {
    private static final double TOLERANCE = 1e-6;

    @Test
    public void testAngleInterpolation() {

        HoodSerializedInformationHolder holder = assertDoesNotThrow(() -> HoodSerializedInformationHolder.create(), "Holder failed to create.");

        assertEquals(1.0, holder.getAngle(1.0), TOLERANCE);
        assertEquals(2.0, holder.getAngle(2.0), TOLERANCE);

        assertEquals(1.5, holder.getAngle(1.5), TOLERANCE);
        assertEquals(1.25, holder.getAngle(1.25), TOLERANCE);

        assertEquals(0.0, holder.getAngle(0), TOLERANCE);
        assertEquals(0.5, holder.getAngle(0.5), TOLERANCE);

        assertEquals(5.0, holder.getAngle(3.5), TOLERANCE);
        assertEquals(6.0, holder.getAngle(4), TOLERANCE);
    }

    @Test
    public void testFlywheelSpeedInterpolation() {
        HoodSerializedInformationHolder holder = assertDoesNotThrow(() -> HoodSerializedInformationHolder.create(), "Holder failed to create.");

        assertEquals(1.0, holder.getFlywheelSpeed(1.0), TOLERANCE);
        assertEquals(2.0, holder.getFlywheelSpeed(2.0), TOLERANCE);

        assertEquals(1.5, holder.getFlywheelSpeed(1.5), TOLERANCE);
        assertEquals(1.25, holder.getFlywheelSpeed(1.25), TOLERANCE);

        assertEquals(0.0, holder.getFlywheelSpeed(0), TOLERANCE);
        assertEquals(0.5, holder.getFlywheelSpeed(0.5), TOLERANCE);

        assertEquals(5.0, holder.getFlywheelSpeed(3.5), TOLERANCE);
        assertEquals(6.0, holder.getFlywheelSpeed(4), TOLERANCE);
    }

    @Test
    public void testAngleInterpolationNonLinear() {
        HoodSerializedInformationHolder holder = assertDoesNotThrow(() -> HoodSerializedInformationHolder.create(), "Holder failed to create.");

        assertEquals(0.5, holder.getAngle(0.5), TOLERANCE);
        assertEquals(1.5, holder.getAngle(1.5), TOLERANCE);
        assertEquals(3, holder.getAngle(2.5), TOLERANCE);
        assertEquals(6, holder.getAngle(4.0), TOLERANCE);
    }

    @Test
    public void testFlywheelSpeedInterpolationNonLinear() {
        HoodSerializedInformationHolder holder = assertDoesNotThrow(() -> HoodSerializedInformationHolder.create(), "Holder failed to create.");

        assertEquals(0.5, holder.getFlywheelSpeed(0.5), TOLERANCE);
        assertEquals(1.5, holder.getFlywheelSpeed(1.5), TOLERANCE);
        assertEquals(3, holder.getFlywheelSpeed(2.5), TOLERANCE);
        assertEquals(6, holder.getFlywheelSpeed(4.0), TOLERANCE);
    }
}
