package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class KickerSubsystem extends SubsystemBase {

    private final double DEFAULT_RUN_SPEED = 0.6;

    private final TalonFX motor;

    public KickerSubsystem() {

        // TODO: device ID
        motor = new TalonFX(Constants.KICKER_PORT.address());

        // to make things easier...
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Runs the motor at the given speed.
     * @param speed speed to run motor at, from [-1.0, 1.0]
     */
    public void run(double speed) {
        motor.set(speed);
    }

    public void run() {
        run(DEFAULT_RUN_SPEED);
    }

    public void stop() {
        motor.stopMotor();
    }
}
