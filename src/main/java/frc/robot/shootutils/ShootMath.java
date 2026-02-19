package frc.robot.shootutils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;

public class ShootMath {

    public static final double G = 9.81;

    private static MutAngle hoodAngle = Units.Revolution.mutable(0);
    private static MutAngularVelocity flywheelSpeed = Units.RevolutionsPerSecond.mutable(0);

    /**
     * Calculates the desired heading of the hood.
     * @param x the current x position of the robot
     * @param y the current y position of the robot
     * @param targetPosX the x position of the target (hub) in the same coordinate system as the robot position
     * @param targetPosY the y position of the target (hub) in the same coordinate system as the robot position
     * @return the heading the hood should aim at, measured from the x-axis, in the current coordinate system
     */
    public static Angle hoodAimAngle(double x, double y, double targetPosX, double targetPosY) {
        return Units.Revolutions.of(Math.atan2(targetPosY - y, targetPosX - x));
    }

    /**
     * Calculates the desired heading of the hood.
     * @param robot a {@link Translation2d} representing the position of the robot
     * @param target a {@link Translation2d} representing the position of the robot
     * @return the heading the hood should aim at, measured from the x-axis, in the current coordinate system
     */
    public static Angle hoodAimAngle(Translation2d robot, Translation2d target) {
        return hoodAimAngle(robot.getX(), robot.getY(), target.getX(), target.getY());
    }

    /**
     * Calculates the needed hood angle to make it into the hub. This uses an empirical calculation.
     * @param x the current x position of the robot
     * @param y the current y position of the robot
     * @param targetPosX the x position of the target (hub) in the same coordinate system as the robot position
     * @param targetPosY the y position of the target (hub) in the same coordinate system as the robot position
     * @return the needed hood angle to make it into the hub
     */
    public static Angle hoodAngle(double x, double y, double targetPosX, double targetPosY) {
        HoodSerializedInformationHolder holder = HoodSerializedInformationHolder.create();
        hoodAngle.mut_replace(Units.Revolutions.of(
                holder.getAngle(Math.pow(Math.pow(x - targetPosX, 2) + Math.pow(y - targetPosY, 2), 0.5))));
        return hoodAngle;
    }

    /**
     * Calculates the needed hood angle to make it into the hub. This uses an empirical calculation.
     * @param robot a {@link Translation2d} representing the position of the robot
     * @param target a {@link Translation2d} representing the position of the robot
     * @return the needed hood angle to make it into the hub
     */
    public static Angle hoodAngle(Translation2d robot, Translation2d target) {
        return hoodAngle(robot.getX(), robot.getY(), target.getX(), target.getY());
    }

    /**
     * Calculates the needed hood angle to make it into the hub. This uses an empirical calculation.
     * @param toTarget a {@link Distance} representing the distance from the robot to the target
     * @return the needed hood angle to make it into the hub
     */
    public static Angle hoodAngle(Distance toTarget) {
        //Assumes we use inches. This can change depending on what coordinate system we use.
        //Pretends that the target is in a straight line away from the target. Because angle doesn't matter, this is fine!
        return hoodAngle(toTarget.in(Units.Inch), 0, 0, 0);
    }

    /**
     * Calculates the needed flywheel speed to make it into the hub. This uses an empirical calculation.
     * @param x the current x position of the robot
     * @param y the current y position of the robot
     * @param targetPosX the x position of the target (hub) in the same coordinate system as the robot position
     * @param targetPosY the y position of the target (hub) in the same coordinate system as the robot position
     * @return the needed flywheel speed to make it into the hub
     */
    public static AngularVelocity flywheelSpeed(double x, double y, double targetPosX, double targetPosY) {
        HoodSerializedInformationHolder holder = HoodSerializedInformationHolder.create();
        flywheelSpeed.mut_replace(Units.RevolutionsPerSecond.of(
                holder.getFlywheelSpeed(Math.pow(Math.pow(x - targetPosX, 2) + Math.pow(y - targetPosY, 2), 0.5))));
        return flywheelSpeed;
    }

    /**
     * Calculates the needed flywheel speed to make it into the hub. This uses an empirical calculation.
     * @param robot a {@link Translation2d} representing the position of the robot
     * @param target a {@link Translation2d} representing the position of the robot
     * @return the needed flywheel speed to make it into the hub
     */
    public static AngularVelocity flywheelSpeed(Translation2d robot, Translation2d target) {
        return flywheelSpeed(robot.getX(), robot.getY(), target.getX(), target.getY());
    }
}
