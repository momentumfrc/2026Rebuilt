package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.motune.MoTuner;
import frc.robot.shootutils.ShootMath;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private PIDController pid;

    public ShooterSubsystem() {

        motor = new TalonFX(Constants.SHOOTER_ADDRESS.address());

        pid = new PIDController(0, 0, 0);

        MoTuner.builder("Shooter PID")
                .p(pid::setP)
                .d(pid::setD)
                .i(pid::setI)
                .iZone(pid::setIZone)
                .parameter("tolerance", pid::setTolerance)
                .measurement(this::getMotorVelocity)
                .safeBuild();
    }

    /**
     * Runs the motor at the speed given by the {@link ShootMath} class.
     * @param robot a <code>Translation2d</code> representing the field-relative position of the robot
     * @param target a <code>Translation2d</code> representing the field-relative position of the target
     */
    public void run(Translation2d robot, Translation2d target) {
        runAtSpeed(ShootMath.flywheelSpeed(robot, target));
    }

    /**
     * Runs the motor at the speed indicated, in terms of rotations per second. This method should not be called outside of this class. Instead, use the <code>run()</code> method.
     * @param speed the speed in rotations per second to run the motor.
     */
    public void runAtSpeed(double rotationsPerSecond) {
        motor.set(pid.calculate(getMotorVelocity(), rotationsPerSecond));
    }

    public boolean isUpToSpeed() {
        return pid.atSetpoint();
    }

    public double getMotorVelocity() {
        return motor.getVelocity().getValueAsDouble();
    }
}
