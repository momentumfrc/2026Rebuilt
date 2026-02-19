package frc.robot;

public class Constants {
    public static final double LOOP_PERIOD = 0.02;

    public static final String TURRET_LIMELIGHT_NAME = "limelight-lembob"; // 10.49.99.12
    public static final String STATIONARY_LIMELIGHT_NAME = "limelight-jimmy"; // 10.49.99.11

    public static final HIDPort DRIVE_CONTORLLER_PORT = new HIDPort(0);
    public static final HIDPort OPERATOR_CONTROLLER_PORT = new HIDPort(1);
    // TODO: actual addresses
    public static final CANAddress KICKER_PORT = new CANAddress(0);
    public static final CANAddress INDEXER_PORT = new CANAddress(0);

    public static final CANAddress HOOD_PORT = new CANAddress(0);

    public static final CANAddress TURRET_MOTOR = new CANAddress(0);
    public static final DIOPort TURRET_ABSOLUTE_ENCODER_1 = new DIOPort(0);
    public static final DIOPort TURRET_ABSOLUTE_ENCODER_2 = new DIOPort(1);

    public static final CANAddress SHOOTER_ADDRESS = new CANAddress(0);

    // Intake Ports
    public static final CANAddress INTAKE_ROLLER_PORT = new CANAddress(0); // edit this with the actual robot
    public static final CANAddress INTAKE_WRIST_PORT = new CANAddress(0);

    public static record CANAddress(int address) {}

    public static record HIDPort(int hidport) {}

    public static record DIOPort(int dioPort) {}

    public static record PWMPort(int port) {}
}
