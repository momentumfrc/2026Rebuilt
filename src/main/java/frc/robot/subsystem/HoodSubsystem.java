package frc.robot.subsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxProfilePID;
import frc.robot.molib.prefs.MoPrefsUtils;
import frc.robot.shootutils.HoodSerializedInformationHolder;
import frc.robot.util.NTHelpers;
import frc.robot.util.SysIdUtil;

@Logged
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

    private final VoltageOut voltageControlRequest = new VoltageOut(0);

    private final BooleanEntry hoodZeroed;
    private final BooleanEntry coastMotor;

    private final BooleanPublisher forwardSoftLimitPublisher;
    private final BooleanPublisher reverseSoftLimitPublisher;
    private final DoublePublisher currentPublisher;
    private final DoublePublisher hoodEncoderPublisher;

    public HoodSubsystem() {
        motor = new TalonFX(Constants.HOOD_PORT.address());
        encoder = MoRotationEncoder.forTalonFx(motor, Units.Revolutions);

        motorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit((Current) MoPrefs.hoodCurrentLimit.get())
                        .withStatorCurrentLimitEnable(true))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withReverseSoftLimitThreshold(
                                MoPrefs.hoodMinSoftLimit.get().in(encoder.getInternalEncoderUnits()))
                        .withReverseSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(
                                MoPrefs.hoodMaxSoftLimit.get().in(encoder.getInternalEncoderUnits()))
                        .withForwardSoftLimitEnable(true));
        motor.getConfigurator().apply(motorConfig);

        MoPrefs.hoodCurrentLimit.subscribe(limit -> {
            motorConfig.CurrentLimits.withStatorCurrentLimit((Current) limit);
            motor.getConfigurator().apply(motorConfig);
        });

        MoPrefs.hoodMaxSoftLimit.subscribe(limit -> {
            motorConfig.SoftwareLimitSwitch.withForwardSoftLimitThreshold(limit.in(encoder.getInternalEncoderUnits()));
            motor.getConfigurator().apply(motorConfig);
        });

        MoPrefs.hoodMinSoftLimit.subscribe(limit -> {
            motorConfig.SoftwareLimitSwitch.withReverseSoftLimitThreshold(limit.in(encoder.getInternalEncoderUnits()));
            motor.getConfigurator().apply(motorConfig);
        });

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

        var table = NTHelpers.getTable("shooter-hood");
        hoodZeroed = NTHelpers.getBooleanEntry(table, "Has zero?", false);
        coastMotor = NTHelpers.getBooleanEntry(table, "Coast Motor", false);
        currentPublisher = table.getDoubleTopic("Current").publish();
        forwardSoftLimitPublisher = table.getBooleanTopic("Forward Soft Limit").publish();
        reverseSoftLimitPublisher = table.getBooleanTopic("Reverse Soft Limit").publish();
        hoodEncoderPublisher = table.getDoubleTopic("Encoder (deg)").publish();
    }

    public void setCalculatedPosition(Distance distance) {
        setPosition(HoodSerializedInformationHolder.getInstance().getHoodAngle(distance));
    }

    private MutAngularVelocity mutVelocityReference = Units.RadiansPerSecond.mutable(0);

    public void setPosition(Angle position) {
        if (Double.isNaN(lastHoodAngle)) {
            lastHoodAngle = position.in(Units.Radians);
        }

        double goalVelocity =
                hoodAngleFilter.calculate((position.in(Units.Radians) - lastHoodAngle) / Constants.LOOP_PERIOD);
        lastHoodAngle = position.in(Units.Radians);

        setPosition(position, mutVelocityReference.mut_replace(goalVelocity, Units.RadiansPerSecond));
    }

    /**
     * Sets the desired position of the hood, in rotations.
     * @param position the desired position of the hood, in rotations
     */
    public void setPosition(Angle position, AngularVelocity velocity) {
        if (hoodZeroed.get() == false) {
            motor.stopMotor();
            return;
        }

        double goalVelocity = velocity.in(Units.RadiansPerSecond);
        State currentState = new State(
                encoder.getPosition().in(Units.Radians), encoder.getVelocity().in(Units.RadiansPerSecond));
        State goalState = new State(position.in(Units.Radians), goalVelocity);
        State setpoint = profile.calculate(Constants.LOOP_PERIOD, currentState, goalState);

        System.out.format("%.4f, %.4f\n", position.in(Units.Degrees), goalState.velocity);

        positionReference.mut_replace(setpoint.position, Units.Radians);
        velocityReference.mut_replace(setpoint.velocity, Units.RadiansPerSecond);
        pid.setReference(positionReference, velocityReference);
    }

    public boolean isInPosition() {
        return hoodZeroed.get() && pid.atSetpoint();
    }

    public void setVoltage(Voltage voltage) {
        voltageControlRequest.withOutput(voltage);
        motor.setControl(voltageControlRequest);
    }

    public Current getCurrent() {
        return motor.getStatorCurrent().getValue();
    }

    public boolean hasZero() {
        return hoodZeroed.get();
    }

    public void disableLimitsForZeroing() {
        motorConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(false);
        motor.getConfigurator().apply(motorConfig);
    }

    public void enableLimits() {
        motorConfig.SoftwareLimitSwitch.withReverseSoftLimitEnable(true);
        motor.getConfigurator().apply(motorConfig);
    }

    public void zeroEncoder() {
        encoder.setPosition(Units.Degrees.zero());
        hoodZeroed.set(true);
    }

    public SysIdRoutine.Mechanism getSysIdMechanism() {
        return SysIdUtil.sysIdMechanismForTalonFx(this, "hood", motor, encoder);
    }

    public Command testCommand(XboxController testController) {
        return run(() -> {
                    double y = -1 * testController.getRightY();
                    double setpointDegrees = MathUtil.interpolate(
                            MoPrefs.hoodMinSoftLimit.get().in(Units.Degrees),
                            MoPrefs.hoodMaxSoftLimit.get().in(Units.Degrees),
                            MathUtil.inverseInterpolate(-1, 1, y));
                    setPosition(Units.Degrees.of(setpointDegrees), Units.RadiansPerSecond.zero());
                })
                .withName("HoodTestCommand");
    }

    @Override
    public void periodic() {
        NeutralModeValue desiredNeutralMode = coastMotor.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        if (desiredNeutralMode != motorConfig.MotorOutput.NeutralMode) {
            motorConfig.MotorOutput.NeutralMode = desiredNeutralMode;
            motor.getConfigurator().apply(motorConfig);
        }

        currentPublisher.set(motor.getStatorCurrent().getValueAsDouble());
        forwardSoftLimitPublisher.set(motor.getFault_ForwardSoftLimit().getValue());
        reverseSoftLimitPublisher.set(motor.getFault_ReverseSoftLimit().getValue());
        hoodEncoderPublisher.set(encoder.getPosition().in(Units.Degrees));
    }
}
