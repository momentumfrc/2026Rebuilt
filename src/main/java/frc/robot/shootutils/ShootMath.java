package frc.robot.shootutils;

public class ShootMath {

    //TODO: put actual values for exit velocity and shooter height

    //The exit velocity of the ball from the hood, in meters per second
    public static final double EXIT_VELOCITY = 5;
    public static final double G = 9.81;
    //The height of the shooter off of the ground, in meters
    public static final double SHOOTER_HEIGHT = 2;

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
     * Calculates the expected rate of rotation of the hood to keep aimed at the target using related rates.
     * @param x the current x position of the robot
     * @param y the current y position of the robot
     * @param targetPosX the x position of the target (hub) in the same coordinate system as the robot position
     * @param targetPosY the y position of the target (hub) in the same coordinate system as the robot position
     * @param xVelocity the signed x velocity
     * @param yVelocity the signed y velocity
     * @return the rate of rotation of the hood to stay aimed at the target
     */
    public static double hoodSwivelSpeed(double x, double y, double targetPosX, double targetPosY, double xVelocity, double yVelocity) {
        double mappedX = x - targetPosX;
        double mappedY = y - targetPosY;
        return (1 / ((Math.pow(mappedX, 2) / Math.pow(mappedY, 2)) + 1)) * ((xVelocity * y - yVelocity * x) / Math.pow(mappedY, 2));
    }

    /**
     * Calculates the needed launch angle to make it into the hub.
     * @param x the current x position of the robot
     * @param y the current y position of the robot
     * @param targetPosX the x position of the target (hub) in the same coordinate system as the robot position
     * @param targetPosY the y position of the target (hub) in the same coordinate system as the robot position
     * @throws IllegalArgumentException thrown if it is not physically possible to make it into the hub
     * @return the needed launch angle to make it into the hub
     */
    public static double launchAngle(double x, double y, double targetPosX, double targetPosY) {
        double mappedY = 72 - SHOOTER_HEIGHT;
        double d = Math.pow(Math.pow(x, 2) + Math.pow(y, 2), 0.5);
        if (Math.pow(EXIT_VELOCITY, 4) < G * (G * Math.pow(d, 2) + 2 * mappedY * Math.pow(EXIT_VELOCITY, 2))) {
            throw new IllegalArgumentException("Not possible to make launch.");
        }
        return Math.atan((Math.pow(EXIT_VELOCITY, 2) + Math.pow(Math.pow(EXIT_VELOCITY, 4) - G * (G * Math.pow(d, 2) + 2 * mappedY * Math.pow(EXIT_VELOCITY, 2)), 0.5)) / (G * d));
    }

}
