package frc.robot;

public class Constants {
    public static final String TURRET_LIMELIGHT_NAME = "limelight-lembob"; // 10.49.99.12
    public static final String STATIONARY_LIMELIGHT_NAME = "limelight-jimmy"; // 10.49.99.11

    public static final HIDPort DRIVE_CONTORLLER_PORT = new HIDPort(0);
    public static final HIDPort OPERATOR_CONTROLLER_PORT = new HIDPort(1);
    // TODO: actual addresses
    public static final CANAddress KICKER_PORT = new CANAddress(0);
    public static final CANAddress INDEXER_PORT = new CANAddress(0);

    public static record CANAddress(int address) {}

    public static record HIDPort(int hidport) {}

    public static record DIOPort(int dioPort) {}

    public static record PWMPort(int port) {}
}
