package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxPID;
import frc.robot.molib.pid.MoTalonFxPID.Type;
import frc.robot.shootutils.HoodSerializedInformationHolder;

public class HoodSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private MoTalonFxPID<AngleUnit, AngularVelocityUnit> pid;

    private MoRotationEncoder encoder;

    public HoodSubsystem() {

        motor = new TalonFX(Constants.HOOD_PORT.address());

        encoder = MoRotationEncoder.forTalonFx(motor, Units.Revolutions);
        encoder.setPosition(Units.Revolutions.of(0));
        MoPrefs.hoodEncoderScale.subscribe(encoder::setConversionFactor, true);

        pid = new MoTalonFxPID<>(Type.POSITION, motor, encoder.getInternalEncoderUnits());

        TunerUtils.forMoTalonFx(pid, "Hood PID");
    }

    public void setCalculatedPosition(Distance distance) {
        setPosition(HoodSerializedInformationHolder.getInstance().getHoodAngle(distance));
    }

    /**
     * Sets the desired position of the hood, in rotations.
     * @param position the desired position of the hood, in rotations
     */
    public void setPosition(Angle position) {
        pid.setPositionReference(position);
    }

    public boolean isInPosition() {
        return pid.atSetpoint();
    }
}
