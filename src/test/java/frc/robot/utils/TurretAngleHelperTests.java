package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.TurretAngleHelper;
import org.junit.jupiter.api.Test;

public class TurretAngleHelperTests {
    @Test
    public void testFullRotation() {
        var angleHelper = new TurretAngleHelper(Rotation2d.kZero, Rotation2d.fromDegrees(360));
        assertEquals(Rotation2d.kZero, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(0)));
        assertEquals(Rotation2d.kCCW_90deg, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(90)));
        assertEquals(Rotation2d.k180deg, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(180)));
        assertEquals(Rotation2d.kCW_90deg, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(270)));

        assertEquals(Rotation2d.fromDegrees(360), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(360)));
        assertEquals(Rotation2d.fromDegrees(1), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(361)));
        assertEquals(Rotation2d.kCCW_90deg, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(450)));
        assertEquals(Rotation2d.kZero, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(720)));
        assertEquals(Rotation2d.kCW_90deg, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-90)));
        assertEquals(Rotation2d.kZero, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-360)));
    }

    @Test
    public void testMultiRotation() {
        var angleHelper = new TurretAngleHelper(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(810));
        assertEquals(Rotation2d.fromDegrees(0), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(0)));
        assertEquals(Rotation2d.fromDegrees(0), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-360)));
        assertEquals(Rotation2d.fromDegrees(0), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-720)));

        assertEquals(Rotation2d.fromDegrees(-90), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-90)));
        assertEquals(Rotation2d.fromDegrees(-90), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-450)));

        assertEquals(Rotation2d.fromDegrees(270), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(270)));
        assertEquals(Rotation2d.fromDegrees(630), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(630)));
        assertEquals(Rotation2d.fromDegrees(630), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(990)));

        assertEquals(Rotation2d.fromDegrees(810), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(810)));
        assertEquals(Rotation2d.fromDegrees(810), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(1170)));
    }

    @Test
    public void testPartialRotation() {
        var angleHelper = new TurretAngleHelper(Rotation2d.fromDegrees(15), Rotation2d.fromDegrees(365));
        assertEquals(Rotation2d.fromDegrees(360), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(0)));
        assertEquals(Rotation2d.fromDegrees(365), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(5)));
        assertNull(angleHelper.turretAngleModulus(Rotation2d.fromDegrees(6)));
        assertNull(angleHelper.turretAngleModulus(Rotation2d.fromDegrees(10)));
        assertNull(angleHelper.turretAngleModulus(Rotation2d.fromDegrees(14)));
        assertEquals(Rotation2d.fromDegrees(15), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(15)));
        assertEquals(Rotation2d.fromDegrees(360), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(360)));
        assertEquals(Rotation2d.fromDegrees(365), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(365)));
        assertNull(angleHelper.turretAngleModulus(Rotation2d.fromDegrees(366)));
        assertNull(angleHelper.turretAngleModulus(Rotation2d.fromDegrees(370)));

        assertEquals(Rotation2d.fromDegrees(270), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-90)));
        assertEquals(Rotation2d.fromDegrees(180), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-180)));
        assertEquals(Rotation2d.fromDegrees(90), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-270)));
        assertEquals(Rotation2d.fromDegrees(15), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-345)));
        assertNull(angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-346)));
        assertNull(angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-350)));
        assertNull(angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-354)));
        assertEquals(Rotation2d.fromDegrees(365), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-355)));
        assertEquals(Rotation2d.fromDegrees(360), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-360)));
        assertEquals(Rotation2d.fromDegrees(355), angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-365)));
    }
}
