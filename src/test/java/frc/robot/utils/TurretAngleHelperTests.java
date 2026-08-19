package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.util.TurretAngleHelper;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;

public class TurretAngleHelperTests {
    private void assertResult(Rotation2d angle, boolean inRange, TurretAngleHelper.Result result) {
        assertEquals(angle, result.angle());
        assertEquals(inRange, result.inRange());
    }

    @Test
    public void testFullRotation() {
        var angleHelper = new TurretAngleHelper(Rotation2d.kZero, Rotation2d.fromDegrees(360));
        assertResult(Rotation2d.kZero, true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(0)));
        assertResult(Rotation2d.kCCW_90deg, true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(90)));
        assertResult(Rotation2d.k180deg, true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(180)));
        assertResult(Rotation2d.kCW_90deg, true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(270)));

        assertResult(Rotation2d.fromDegrees(360), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(360)));
        assertResult(Rotation2d.fromDegrees(1), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(361)));
        assertResult(Rotation2d.kCCW_90deg, true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(450)));
        assertResult(Rotation2d.kZero, true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(720)));
        assertResult(Rotation2d.kCW_90deg, true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-90)));
        assertResult(Rotation2d.kZero, true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-360)));
    }

    @Test
    public void testPartialRotation() {
        var angleHelper = new TurretAngleHelper(Rotation2d.fromDegrees(15), Rotation2d.fromDegrees(365));
        assertResult(Rotation2d.fromDegrees(360), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(0)));
        assertResult(Rotation2d.fromDegrees(365), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(5)));

        assertResult(Rotation2d.fromDegrees(330), false, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(6)));
        assertResult(Rotation2d.fromDegrees(190), false, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(10)));
        assertResult(Rotation2d.fromDegrees(50), false, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(14)));

        assertResult(Rotation2d.fromDegrees(15), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(15)));
        assertResult(Rotation2d.fromDegrees(360), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(360)));
        assertResult(Rotation2d.fromDegrees(365), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(365)));

        assertResult(Rotation2d.fromDegrees(330), false, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(366)));
        assertResult(Rotation2d.fromDegrees(190), false, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(370)));

        assertResult(Rotation2d.fromDegrees(270), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-90)));
        assertResult(Rotation2d.fromDegrees(180), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-180)));
        assertResult(Rotation2d.fromDegrees(90), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-270)));
        assertResult(Rotation2d.fromDegrees(15), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-345)));

        assertResult(Rotation2d.fromDegrees(50), false, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-346)));
        assertResult(Rotation2d.fromDegrees(190), false, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-350)));
        assertResult(Rotation2d.fromDegrees(330), false, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-354)));

        assertResult(Rotation2d.fromDegrees(365), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-355)));
        assertResult(Rotation2d.fromDegrees(360), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-360)));
        assertResult(Rotation2d.fromDegrees(355), true, angleHelper.turretAngleModulus(Rotation2d.fromDegrees(-365)));
    }
}
