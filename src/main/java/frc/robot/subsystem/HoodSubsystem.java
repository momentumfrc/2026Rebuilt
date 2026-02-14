package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxPID;
import frc.robot.molib.pid.MoTalonFxPID.Type;
import frc.robot.shootutils.ShootMath;

public class HoodSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private MoTalonFxPID<AngleUnit, AngularVelocityUnit> pid;

    private MoRotationEncoder encoder;

    public HoodSubsystem() {

        motor = new TalonFX(Constants.HOOD_PORT.address());

        encoder = MoRotationEncoder.forTalonFx(motor, Units.Revolutions);
        encoder.setPosition(Units.Revolutions.of(0));
        encoder.setConversionFactor(MoPrefs.hoodEncoderScale.get());

        pid = new MoTalonFxPID<>(Type.POSITION, motor, encoder.getInternalEncoderUnits());

        TunerUtils.forMoTalonFx(pid, "Hood PID");
    }

    /**
     * Sets the hood position calculated by {@link ShootMath}.
     * @param robot the position of the robot relative to the field
     * @param target the position of the target relative to the field
     */
    public void setCalculatedPosition(Translation2d robot, Translation2d target) {
        setPosition(ShootMath.hoodAimAngle(robot, target));
    }

    /**
     * Sets the desired position of the hood, in rotations.
     * @param position the desired position of the hood, in rotations
     */
    public void setPosition(Angle position) {
        pid.setPositionReference(position);
    }

    public boolean isInPosition() {
        return Units.Revolutions.of(encoder.getPosition()
                                .minus(Units.Revolution.of(pid.getSetpoint()))
                                .abs(Units.Revolution))
                        .compareTo(MoPrefs.hoodTolerance.get())
                < 0;
    }
}
