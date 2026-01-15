package frc.robot;

public class Constants {
    public static final HIDPort DRIVE_CONTORLLER_PORT = new HIDPort(0);

    public static final double DEADBAND = 0.05;

    public static record CANAddress(int address) {}

    public static record HIDPort(int hidport) {}

    public static record PWMPort(int port) {}
}
