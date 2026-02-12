package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.MoTuner;
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

        pid = new MoTalonFxPID<>(Type.POSITION, motor, encoder.getInternalEncoderUnits());

        MoTuner.builder("Hood PID")
                .d(pid::setD)
                .i(pid::setI)
                .p(pid::setP)
                .iZone(pid::setIZone)
                .measurement(encoder::getPositionInEncoderUnits)
                .safeBuild();
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
    public void setPosition(double position) {
        pid.setPositionReference(Units.Revolutions.of(position));
    }
}
