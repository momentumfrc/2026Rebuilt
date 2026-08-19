package first.subsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import first.Constants;
import first.MoPrefs;
import first.molib.NTHelpers;
import first.molib.encoder.MoRotationEncoder;
import first.molib.motune.TunerUtils;
import first.molib.pid.MoTalonFxProfilePID;
import first.molib.prefs.MoPrefsUtils;
import first.shootutils.HoodSerializedInformationHolder;
import first.util.SysIdUtil;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.State;
import org.wpilib.math.util.MathUtil;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;

public class HoodSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final TalonFXConfiguration motorConfig;

    private final MoTalonFxProfilePID<AngleUnit, AngularVelocityUnit> pid;
    private final LinearFilter hoodAngleFilter = LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_PERIOD));
    private TrapezoidProfile profile;
    private State setpointState = new State();

    private double lastHoodAngle = Double.NaN;
    private final MoRotationEncoder encoder;

    private final VoltageOut voltageControlRequest = new VoltageOut(0);

    private final BooleanEntry hoodZeroed;
    private final BooleanEntry coastMotor;

    private final BooleanPublisher forwardSoftLimitPublisher;
    private final BooleanPublisher reverseSoftLimitPublisher;
    private final DoublePublisher currentPublisher;
    private final DoublePublisher hoodEncoderPublisher;

    private final DoublePublisher calculatedHoodPositionPublisher;

    public HoodSubsystem() {
        motor = new TalonFX(Constants.HOOD_PORT.address(), CANBus.systemcore(Constants.DEFAULT_CAN_BUS));
        motorConfig = new TalonFXConfiguration();
        encoder = MoRotationEncoder.forTalonFx(motor, Units.Revolutions, motorConfig);

        motorConfig
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

        pid = new MoTalonFxProfilePID<>(motor, encoder.getInternalEncoderUnits(), motorConfig);

        TunerUtils.forMoTalonFxProfile(pid, "Hood PID");

        var table = NTHelpers.getTable("shooter-hood");
        hoodZeroed = NTHelpers.getBooleanEntry(table, "Has zero?", false);
        coastMotor = NTHelpers.getBooleanEntry(table, "Coast Motor", false);
        currentPublisher = table.getDoubleTopic("Current").publish();
        forwardSoftLimitPublisher = table.getBooleanTopic("Forward Soft Limit").publish();
        reverseSoftLimitPublisher = table.getBooleanTopic("Reverse Soft Limit").publish();
        hoodEncoderPublisher = table.getDoubleTopic("Encoder (deg)").publish();

        calculatedHoodPositionPublisher =
                table.getDoubleTopic("Calculated Hood Position (deg)").publish();
    }

    public void setCalculatedPosition(Distance distance) {
        Angle position = HoodSerializedInformationHolder.getInstance().getHoodAngle(distance);
        calculatedHoodPositionPublisher.set(position.in(Units.Degrees));
        setPosition(position);
    }

    // TODO
    // private MutAngularVelocity mutVelocityReference = Units.RadiansPerSecond.mutable(0);

    public void goToRest() {
        setPosition(MoPrefs.hoodDeadzonePosition.get(), Units.DegreesPerSecond.zero());
        lastHoodAngle = Double.NaN;
    }

    public void setPosition(Angle position) {
        if (Double.isNaN(lastHoodAngle)) {
            hoodAngleFilter.reset();
            lastHoodAngle = position.in(Units.Radians);
        }

        double goalVelocity =
                hoodAngleFilter.calculate((position.in(Units.Radians) - lastHoodAngle) / Constants.LOOP_PERIOD);
        lastHoodAngle = position.in(Units.Radians);

        // TODO
        //setPosition(position, mutVelocityReference.mut_replace(goalVelocity, Units.RadiansPerSecond));
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
        State goalState = new State(position.in(Units.Radians), goalVelocity);
        setpointState = profile.calculate(Constants.LOOP_PERIOD, setpointState, goalState);

        // TODO
        // positionReference.mut_replace(setpointState.position, Units.Radians);
        // velocityReference.mut_replace(setpointState.velocity, Units.RadiansPerSecond);
        // pid.setReference(positionReference, velocityReference);
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

    public Command testCommand(Gamepad testController) {
        return run(() -> {
                    double y = -1 * testController.getRightY();
                    double setpointDegrees = MathUtil.lerp(
                            MoPrefs.hoodMinSoftLimit.get().in(Units.Degrees),
                            MoPrefs.hoodMaxSoftLimit.get().in(Units.Degrees),
                            MathUtil.inverseLerp(-1, 1, y));
                    // TODO
                    // positionReference.mut_replace(setpointDegrees, Units.Degrees);
                    // pid.setReference(positionReference, Units.DegreesPerSecond.zero());
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
