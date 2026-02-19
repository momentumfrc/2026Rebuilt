package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxPID;
import frc.robot.molib.pid.MoTalonFxPID.Type;
import frc.robot.shootutils.HoodSerializedInformationHolder;

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
     * Runs the motor at the speed indicated.
     * @param speed the speed to run the motor.
     */
    public void runAtSpeed(AngularVelocity velocity) {
        pid.setVelocityReference(velocity);
    }

    public void runAtCalculatedSpeed(Distance distanceToTarget) {
        runAtSpeed(HoodSerializedInformationHolder.getInstance().getFlywheelSpeed(distanceToTarget));
    }

    public void stop() {
        runAtSpeed(Units.RPM.zero());
    }

    public boolean isUpToSpeed() {
        return pid.atSetpoint();
    }

    public double getMotorVelocity() {
        return encoder.getVelocityInEncoderUnits();
    }
}
