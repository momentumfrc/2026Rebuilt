package frc.robot.shootutils;

public class ShootMath {

    //The exit velocity of the ball from the hood
    public static final double EXIT_VELOCITY = 5;

    public static double hoodAimAngle(double x, double y, double targetPosX, double targetPosY) {
        return Math.atan2(targetPosY - y, targetPosX - x);
    }

}
