package frc.robot.shootutils;

import edu.wpi.first.math.geometry.Translation2d;

public class ShootMath {

    public static final double G = 9.81;

    /**
     * Calculates the desired heading of the hood.
     * @param x the current x position of the robot
     * @param y the current y position of the robot
     * @param targetPosX the x position of the target (hub) in the same coordinate system as the robot position
     * @param targetPosY the y position of the target (hub) in the same coordinate system as the robot position
     * @return the heading the hood should aim at, measured from the x-axis, in the current coordinate system
     */
    public static double hoodAimAngle(double x, double y, double targetPosX, double targetPosY) {
        return Math.atan2(targetPosY - y, targetPosX - x);
    }

    /**
     * Calculates the desired heading of the hood.
     * @param robot a {@link Translation2d} representing the position of the robot
     * @param target a {@link Translation2d} representing the position of the robot
     * @return the heading the hood should aim at, measured from the x-axis, in the current coordinate system
     */
    public static double hoodAimAngle(Translation2d robot, Translation2d target) {
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
    public static double hoodAngle(double x, double y, double targetPosX, double targetPosY) {
        HoodSerializedInformationHolder holder = HoodSerializedInformationHolder.create();
        return holder.getAngle(Math.pow(Math.pow(x - targetPosX, 2) + Math.pow(y - targetPosY, 2), 0.5));
    }

    /**
     * Calculates the needed hood angle to make it into the hub. This uses an empirical calculation.
     * @param robot a {@link Translation2d} representing the position of the robot
     * @param target a {@link Translation2d} representing the position of the robot
     * @return the needed hood angle to make it into the hub
     */
    public static double hoodAngle(Translation2d robot, Translation2d target) {
        return hoodAngle(robot.getX(), robot.getY(), target.getX(), target.getY());
    }

    /**
     * Calculates the needed flywheel speed to make it into the hub. This uses an empirical calculation.
     * @param x the current x position of the robot
     * @param y the current y position of the robot
     * @param targetPosX the x position of the target (hub) in the same coordinate system as the robot position
     * @param targetPosY the y position of the target (hub) in the same coordinate system as the robot position
     * @return the needed flywheel speed to make it into the hub
     */
    public static double flywheelSpeed(double x, double y, double targetPosX, double targetPosY) {
        HoodSerializedInformationHolder holder = HoodSerializedInformationHolder.create();
        return holder.getFlywheelSpeed(Math.pow(Math.pow(x - targetPosX, 2) + Math.pow(y - targetPosY, 2), 0.5));
    }

    /**
     * Calculates the needed flywheel speed to make it into the hub. This uses an empirical calculation.
     * @param robot a {@link Translation2d} representing the position of the robot
     * @param target a {@link Translation2d} representing the position of the robot
     * @return the needed flywheel speed to make it into the hub
     */
    public static double flywheelSpeed(Translation2d robot, Translation2d target) {
        return flywheelSpeed(robot.getX(), robot.getY(), target.getX(), target.getY());
    }
}
