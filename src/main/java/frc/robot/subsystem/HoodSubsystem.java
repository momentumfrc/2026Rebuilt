package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.motune.MoTuner;

public class HoodSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private PIDController pid;

    private double position;

    public HoodSubsystem() {

        motor = new TalonFX(Constants.HOOD_PORT.address());

        motor.setPosition(0);

        pid = new PIDController(0, 0, 0);

        MoTuner.builder("Hood PID").d(pid::setD).i(pid::setI).p(pid::setP).iZone(pid::setIZone).measurement(motor.getPosition()::getValueAsDouble).parameter("tolerance", pid::setTolerance).safeBuild();

    }

    /**
     * Sets the desired position of the hood, in rotations.
     * @param position the desired position of the hood, in rotations
     */
    public void setPosition(double position) {
        this.position = position;
    }

    /**
     * Moves to the desired position.
     */
    public void moveToPosition() {
        motor.set(pid.calculate(motor.getPosition().getValueAsDouble(), position));
    }

}
