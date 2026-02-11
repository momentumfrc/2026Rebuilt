package frc.robot;

public class Constants {
    public static final HIDPort DRIVE_CONTORLLER_PORT = new HIDPort(0);
    // TODO: actual addresses
    public static final CANAddress KICKER_PORT = new CANAddress(0);
    public static final CANAddress INDEXER_PORT = new CANAddress(0);

    public static record CANAddress(int address) {}

    public static record HIDPort(int hidport) {}

    public static record PWMPort(int port) {}
}
