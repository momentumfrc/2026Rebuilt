package frc.robot;

public class Constants {
    public static final String TURRET_LIMELIGHT_NAME = "limelight-turret";
    public static final String STATIONARY_LIMELIGHT_NAME = "limelight-odom";

    public static final HIDPort DRIVE_CONTORLLER_PORT = new HIDPort(0);

    public static record CANAddress(int address) {}

    public static record HIDPort(int hidport) {}

    public static record PWMPort(int port) {}
}
