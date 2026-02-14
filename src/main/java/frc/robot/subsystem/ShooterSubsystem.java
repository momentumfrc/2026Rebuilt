package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.MoTuner;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxPID;
import frc.robot.molib.pid.MoTalonFxPID.Type;
import frc.robot.shootutils.ShootMath;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private MoTalonFxPID<AngleUnit, AngularVelocityUnit> pid;
    private MoRotationEncoder encoder;

    public ShooterSubsystem() {

        motor = new TalonFX(Constants.SHOOTER_ADDRESS.address());

        encoder = MoRotationEncoder.forTalonFx(motor, Units.Revolutions);
        pid = new MoTalonFxPID<>(Type.POSITION, motor, encoder.getInternalEncoderUnits());

        TunerUtils.forMoTalonFx(pid, "Shooter PID");
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
        pid.setVelocityReference(Units.RevolutionsPerSecond.of(rotationsPerSecond));
    }

    public boolean isUpToSpeed() {
        return Math.abs(getMotorVelocity() - pid.getSetpoint())
                < MoPrefs.flywheelSpeedTolerance.get().baseUnitMagnitude();
    }

    public double getMotorVelocity() {
        return encoder.getVelocityInEncoderUnits();
    }
}
