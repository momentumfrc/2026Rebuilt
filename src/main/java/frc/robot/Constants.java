package frc.robot;

public class Constants {
    public static final HIDPort DRIVE_CONTORLLER_PORT = new HIDPort(0);
    public static final HIDPort OPERATOR_CONTROLLER_PORT = new HIDPort(1);

    // Intake Ports
    public static final CANAddress INTAKE_ROLLER_PORT = new CANAddress(0); // edit this with the actual robot
    public static final CANAddress INTAKE_WRIST_PORT = new CANAddress(0);

    public static record CANAddress(int address) {}

    public static record HIDPort(int hidport) {}

    public static record PWMPort(int port) {}
}
