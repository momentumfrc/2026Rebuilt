package frc.robot.subsystem;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxProfilePID;
import frc.robot.molib.prefs.MoPrefsUtils;
import frc.robot.shootutils.HoodSerializedInformationHolder;

public class HoodSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final TalonFXConfiguration motorConfig;

    private final MoTalonFxProfilePID<AngleUnit, AngularVelocityUnit> pid;
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_PERIOD));
    private TrapezoidProfile profile;

    private double lastHoodAngle = Double.NaN;
    private final MoRotationEncoder encoder;
    private final MutAngle positionReference = Units.Radians.mutable(0);
    private final MutAngularVelocity velocityReference = Units.RadiansPerSecond.mutable(0);

    public HoodSubsystem() {
        motor = new TalonFX(Constants.HOOD_PORT.address());
        motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withReverseSoftLimitThreshold(0)
                        .withReverseSoftLimitEnable(false)
                        .withForwardSoftLimitThreshold(MoPrefs.hoodMaxSoftLimit.get())
                        .withForwardSoftLimitEnable(true));
        motor.getConfigurator().apply(motorConfig);

        MoPrefs.hoodMaxSoftLimit.subscribe(limit -> {
            motorConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold((Angle) limit);
            motor.getConfigurator().apply(motorConfig);
        });

        encoder = MoRotationEncoder.forTalonFx(motor, Units.Revolutions);
        encoder.setPosition(Units.Revolutions.of(0));
        MoPrefs.hoodEncoderScale.subscribe(encoder::setConversionFactor, true);

        MoPrefsUtils.multiSubscribe(
                MoPrefs.hoodMaxVelocity,
                MoPrefs.hoodMaxAcceleration,
                (maxVel, maxAcc) -> {
                    this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                            maxVel.in(Units.RadiansPerSecond), maxAcc.in(Units.RadiansPerSecondPerSecond)));
                },
                true);

        pid = new MoTalonFxProfilePID<>(motor, encoder.getInternalEncoderUnits());

        TunerUtils.forMoTalonFxProfile(pid, "Hood PID");
    }

    public void setCalculatedPosition(Distance distance) {
        setPosition(HoodSerializedInformationHolder.getInstance().getHoodAngle(distance));
    }

    /**
     * Sets the desired position of the hood, in rotations.
     * @param position the desired position of the hood, in rotations
     */
    public void setPosition(Angle position) {
        if (Double.isNaN(lastHoodAngle)) {
            lastHoodAngle = position.in(Units.Radians);
        }

        double goalVelocity =
                hoodAngleFilter.calculate((position.in(Units.Radians) - lastHoodAngle) / Constants.LOOP_PERIOD);
        State currentState = new State(
                encoder.getPosition().in(Units.Radians), encoder.getVelocity().in(Units.RadiansPerSecond));
        State goalState = new State(position.in(Units.Radians), goalVelocity);
        State setpoint = profile.calculate(Constants.LOOP_PERIOD, currentState, goalState);

        positionReference.mut_replace(setpoint.position, Units.Radians);
        velocityReference.mut_replace(setpoint.velocity, Units.RadiansPerSecond);
        pid.setReference(positionReference, velocityReference);
    }

    public boolean isInPosition() {
        return pid.atSetpoint();
    }
}
