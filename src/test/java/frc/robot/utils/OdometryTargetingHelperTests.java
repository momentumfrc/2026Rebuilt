package frc.robot.utils;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.OdometryTargetingHelper;
import org.junit.jupiter.api.Test;

public class OdometryTargetingHelperTests {
    @Test
    public void testAngleInlineYAxis() {
        var rightInFront = new Translation2d(2, 4.021328);
        assertEquals(
                Rotation2d.kZero,
                OdometryTargetingHelper.getTranslationToTarget(rightInFront, DriverStation.Alliance.Blue)
                        .getAngle());
        assertEquals(
                Rotation2d.kZero,
                OdometryTargetingHelper.getTranslationToTarget(rightInFront, DriverStation.Alliance.Red)
                        .getAngle());

        var betweenTargets = new Translation2d(6, 4.021328);
        assertEquals(
                Rotation2d.k180deg,
                OdometryTargetingHelper.getTranslationToTarget(betweenTargets, DriverStation.Alliance.Blue)
                        .getAngle());
        assertEquals(
                Rotation2d.kZero,
                OdometryTargetingHelper.getTranslationToTarget(betweenTargets, DriverStation.Alliance.Red)
                        .getAngle());

        var behindBoth = new Translation2d(13, 4.021328);
        assertEquals(
                Rotation2d.k180deg,
                OdometryTargetingHelper.getTranslationToTarget(behindBoth, DriverStation.Alliance.Blue)
                        .getAngle());
        assertEquals(
                Rotation2d.k180deg,
                OdometryTargetingHelper.getTranslationToTarget(behindBoth, DriverStation.Alliance.Red)
                        .getAngle());
    }

    @Test
    public void testAngleInlineXAxis() {
        var leftBlue = new Translation2d(4.60502, 8);
        assertEquals(
                Rotation2d.kCW_90deg,
                OdometryTargetingHelper.getTranslationToTarget(leftBlue, DriverStation.Alliance.Blue)
                        .getAngle());

        var leftRed = new Translation2d(11.908028, 8);
        assertEquals(
                Rotation2d.kCW_90deg,
                OdometryTargetingHelper.getTranslationToTarget(leftRed, DriverStation.Alliance.Red)
                        .getAngle());

        var rightBlue = new Translation2d(4.60502, 2);
        assertEquals(
                Rotation2d.kCCW_90deg,
                OdometryTargetingHelper.getTranslationToTarget(rightBlue, DriverStation.Alliance.Blue)
                        .getAngle());

        var rightRed = new Translation2d(11.908028, 2);
        assertEquals(
                Rotation2d.kCCW_90deg,
                OdometryTargetingHelper.getTranslationToTarget(rightRed, DriverStation.Alliance.Red)
                        .getAngle());
    }
}
