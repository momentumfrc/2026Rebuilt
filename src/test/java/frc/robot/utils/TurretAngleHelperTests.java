package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import first.robot.util.TurretAngleHelper;
import org.junit.jupiter.api.Test;
import org.wpilib.units.measure.Angle;

import static org.wpilib.units.Units.Degrees;

public class TurretAngleHelperTests {
    private void assertResult(Angle angle, boolean inRange, TurretAngleHelper.Result result) {
        assertEquals(angle, result.angle(), angle.in(Degrees) + " != " + result.angle().in(Degrees));
        assertEquals(inRange, result.inRange());
    }

    @Test
    public void testFullRotation() {
        var angleHelper = new TurretAngleHelper(Degrees.zero(), Degrees.of(360));
        assertResult(Degrees.zero(), true, angleHelper.turretAngleModulus(Degrees.of(0)));
        assertResult(Degrees.of(90), true, angleHelper.turretAngleModulus(Degrees.of(90)));
        assertResult(Degrees.of(180), true, angleHelper.turretAngleModulus(Degrees.of(180)));
        assertResult(Degrees.of(270), true, angleHelper.turretAngleModulus(Degrees.of(270)));

        assertResult(Degrees.of(0), true, angleHelper.turretAngleModulus(Degrees.of(360)));
        assertResult(Degrees.of(1), true, angleHelper.turretAngleModulus(Degrees.of(361)));
        assertResult(Degrees.of(90), true, angleHelper.turretAngleModulus(Degrees.of(450)));
        assertResult(Degrees.zero(), true, angleHelper.turretAngleModulus(Degrees.of(720)));
        assertResult(Degrees.of(270), true, angleHelper.turretAngleModulus(Degrees.of(-90)));
        assertResult(Degrees.zero(), true, angleHelper.turretAngleModulus(Degrees.of(-360)));
    }

    @Test
    public void testPartialRotation() {
        var angleHelper = new TurretAngleHelper(Degrees.of(15), Degrees.of(365));
        assertResult(Degrees.of(360), true, angleHelper.turretAngleModulus(Degrees.of(0)));
        assertResult(Degrees.of(365), true, angleHelper.turretAngleModulus(Degrees.of(5)));

        assertResult(Degrees.of(330), false, angleHelper.turretAngleModulus(Degrees.of(6)));
        assertResult(Degrees.of(190), false, angleHelper.turretAngleModulus(Degrees.of(10)));
        assertResult(Degrees.of(50), false, angleHelper.turretAngleModulus(Degrees.of(14)));

        assertResult(Degrees.of(15), true, angleHelper.turretAngleModulus(Degrees.of(15)));
        assertResult(Degrees.of(360), true, angleHelper.turretAngleModulus(Degrees.of(360)));
        assertResult(Degrees.of(365), true, angleHelper.turretAngleModulus(Degrees.of(365)));

        assertResult(Degrees.of(330), false, angleHelper.turretAngleModulus(Degrees.of(366)));
        assertResult(Degrees.of(190), false, angleHelper.turretAngleModulus(Degrees.of(370)));

        assertResult(Degrees.of(270), true, angleHelper.turretAngleModulus(Degrees.of(-90)));
        assertResult(Degrees.of(180), true, angleHelper.turretAngleModulus(Degrees.of(-180)));
        assertResult(Degrees.of(90), true, angleHelper.turretAngleModulus(Degrees.of(-270)));
        assertResult(Degrees.of(15), true, angleHelper.turretAngleModulus(Degrees.of(-345)));

        assertResult(Degrees.of(50), false, angleHelper.turretAngleModulus(Degrees.of(-346)));
        assertResult(Degrees.of(190), false, angleHelper.turretAngleModulus(Degrees.of(-350)));
        assertResult(Degrees.of(330), false, angleHelper.turretAngleModulus(Degrees.of(-354)));

        assertResult(Degrees.of(365), true, angleHelper.turretAngleModulus(Degrees.of(-355)));
        assertResult(Degrees.of(360), true, angleHelper.turretAngleModulus(Degrees.of(-360)));
        assertResult(Degrees.of(355), true, angleHelper.turretAngleModulus(Degrees.of(-365)));
    }
}
