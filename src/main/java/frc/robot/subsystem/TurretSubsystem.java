package frc.robot.subsystem;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.encoder.absolute.MoAbsoluteEncoder;
import frc.robot.molib.encoder.absolute.VernierEncoder;
import frc.robot.molib.motune.MoTuner;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxProfilePID;
import frc.robot.molib.prefs.MoPrefsUtils;
import frc.robot.shootutils.TurretTargeting.TurretSetpoint;
import frc.robot.util.LimelightTargetingHelper;
import frc.robot.util.NTHelpers;
import frc.robot.util.TurretAngleHelper;

public class TurretSubsystem extends SubsystemBase {
    private static final int MAIN_GEAR_TOOTH_COUNT = 85;
    private static final int ENCODER_1_GEAR_TOOTH_COUNT = 15;
    private static final int ENCODER_2_GEAR_TOOTH_COUNT = 16;

    public static final Transform3d robotToTurret = new Transform3d(-0.154305, -0.031750, 0.381, Rotation3d.kZero);
    public static final Transform3d turretToCamera =
            new Transform3d(0.181656, 0, 0.146352, new Rotation3d(0, degreesToRadians(15), 0));

    private final TalonFX turretMotor;
    private final TalonFXConfiguration turretMotorConfig;
    private final MoRotationEncoder turretEncoder;

    public static class TimestampedEncoderReading {
        private MutAngle value = Units.Radians.mutable(0);
        private double timestamp;

        public Angle value() {
            return value;
        }

        public double timestamp() {
            return timestamp;
        }
    }

    private TimestampedEncoderReading timestampedTurretYaw = new TimestampedEncoderReading();

    /*
     * Notes about the encoders.
     * <ul>
     * <li> Positive rotation is counter-clockwise. This applies to all encoders and the motor.
     * <li> The absolute encoder is zeroed at the clockwise-most position of the mechanism.
     * <li> The relative encoder is zeroed pointing forward on the robot (towards the intake).
     * </ul>
     */
    private final MoAbsoluteEncoder absEncoder1;
    private final MoAbsoluteEncoder absEncoder2;
    private final VernierEncoder vernierEncoder;
    private TurretAngleHelper angleHelper;

    private TrapezoidProfile profile;
    private final MoTalonFxProfilePID<AngleUnit, AngularVelocityUnit> turretAbsolutePid;
    private final PIDController turretRelativePid;

    private final MutAngle goalAngle = Units.Rotations.mutable(0);
    private final MutAngularVelocity goalVelocity = Units.RPM.mutable(0);

    private final LimelightTargetingHelper targetingHelper;
    private final DoublePublisher relativeEncoderPublisher;
    private final DoublePublisher absEncoder1Publisher;
    private final DoublePublisher absEncoder2Publisher;
    private final DoublePublisher vernierEncoderPublisher;
    private final IntegerPublisher targetTagPublisher;

    private final BooleanEntry coastMotorEntry;

    public TurretSubsystem() {

        /* ==== MOTOR SETUP === */
        this.turretMotor = new TalonFX(Constants.TURRET_MOTOR.address());
        this.turretMotorConfig = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withReverseSoftLimitThreshold(MoPrefs.turretMinSoftLimit.get())
                        .withReverseSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(MoPrefs.turretMaxSoftLimit.get())
                        .withForwardSoftLimitEnable(true))
                .withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage((Voltage) MoPrefs.turretMaxPower.get())
                        .withPeakReverseVoltage((Voltage) MoPrefs.turretMaxPower.get()))
                .withClosedLoopRamps(
                        new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(MoPrefs.turretVoltRampRate.get()))
                .withOpenLoopRamps(
                        new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(MoPrefs.turretVoltRampRate.get()));
        turretMotor.getConfigurator().apply(turretMotorConfig);

        MoPrefsUtils.multiSubscribe(MoPrefs.turretMinSoftLimit, MoPrefs.turretMaxSoftLimit, (min, max) -> {
            turretMotorConfig
                    .SoftwareLimitSwitch
                    .withReverseSoftLimitThreshold((Angle) min)
                    .withForwardSoftLimitThreshold((Angle) max);
            turretMotor.getConfigurator().apply(turretMotorConfig);
        });

        MoPrefs.turretMaxPower.subscribe(voltage -> {
            turretMotorConfig.Voltage.withPeakForwardVoltage((Voltage) voltage).withPeakReverseVoltage((Voltage)
                    voltage);
            turretMotor.getConfigurator().apply(turretMotorConfig);
        });

        MoPrefs.turretVoltRampRate.subscribe(rampRate -> {
            turretMotorConfig.ClosedLoopRamps.withVoltageClosedLoopRampPeriod((Time) rampRate);
            turretMotorConfig.OpenLoopRamps.withVoltageOpenLoopRampPeriod((Time) rampRate);
            turretMotor.getConfigurator().apply(turretMotorConfig);
        });

        /* ==== ENCODER SETUP ==== */
        MoPrefsUtils.multiSubscribe(
                MoPrefs.turretMinSoftLimit,
                MoPrefs.turretMaxSoftLimit,
                (min, max) -> {
                    angleHelper = new TurretAngleHelper(
                            Rotation2d.fromRadians(min.in(Units.Radians)),
                            Rotation2d.fromRadians(max.in(Units.Radians)));
                },
                true);

        this.turretEncoder = MoRotationEncoder.forTalonFx(turretMotor, Units.Degrees);
        MoPrefs.turretRelativeEncoderScale.subscribe(turretEncoder::setConversionFactor, true);

        this.absEncoder1 = MoAbsoluteEncoder.forDio(Constants.TURRET_ABSOLUTE_ENCODER_1.dioPort());
        this.absEncoder2 = MoAbsoluteEncoder.forDio(Constants.TURRET_ABSOLUTE_ENCODER_2.dioPort());
        this.vernierEncoder = new VernierEncoder(
                absEncoder1,
                absEncoder2,
                new VernierEncoder.GearRatios(
                        MAIN_GEAR_TOOTH_COUNT, ENCODER_1_GEAR_TOOTH_COUNT, ENCODER_2_GEAR_TOOTH_COUNT));
        MoPrefsUtils.multiSubscribe(
                MoPrefs.turretEncoder1Zero,
                MoPrefs.turretEncoder2Zero,
                MoPrefs.turretRelativeEncoderOffset,
                (zero1, zero2, offset) -> {
                    absEncoder1.setEncoderZero((Angle) zero1);
                    absEncoder2.setEncoderZero((Angle) zero2);
                    turretEncoder.setPosition(vernierEncoder.getPosition().plus(offset));
                },
                true);

        /* ==== PID SETUP ==== */
        MoPrefsUtils.multiSubscribe(
                MoPrefs.turretMaxVelocity,
                MoPrefs.turretMaxAcceleration,
                (maxVel, maxAcc) -> {
                    this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                            maxVel.in(Units.RadiansPerSecond), maxAcc.in(Units.RadiansPerSecondPerSecond)));
                },
                true);

        this.turretAbsolutePid = new MoTalonFxProfilePID<AngleUnit, AngularVelocityUnit>(
                turretMotor, turretEncoder.getInternalEncoderUnits());
        TunerUtils.forMoTalonFxProfile(turretAbsolutePid, "Turret Absolute Position");

        this.targetingHelper = new LimelightTargetingHelper(Constants.TURRET_LIMELIGHT_NAME);

        this.turretRelativePid = new PIDController(0, 0, 0);
        MoTuner.builder("Turret Relative Alignment")
                .p(turretRelativePid::setP)
                .i(turretRelativePid::setI)
                .d(turretRelativePid::setD)
                .iZone(turretRelativePid::setIZone)
                .parameter("tolerance", turretRelativePid::setTolerance)
                .measurement(targetingHelper::getTx)
                .safeBuild();

        /* ==== DASHBOARD SETUP ==== */
        var table = NTHelpers.getTable("turret");
        relativeEncoderPublisher = table.getDoubleTopic("Relative Encoder").publish();
        absEncoder1Publisher = table.getDoubleTopic("Abs Encoder 1").publish();
        absEncoder2Publisher = table.getDoubleTopic("Abs Encoder 2").publish();
        vernierEncoderPublisher = table.getDoubleTopic("Vernier Encoder").publish();
        targetTagPublisher = table.getIntegerTopic("Target Tag ID").publish();

        coastMotorEntry = NTHelpers.getBooleanEntry(table, "Coast Motor", false);
    }

    public TurretAngleHelper getAngleHelper() {
        return angleHelper;
    }

    /**
     * Get the current angular position of the turret about its axis of rotation.
     */
    public Angle getTurretYaw() {
        return turretEncoder.getPosition();
    }

    public TimestampedEncoderReading getTimestampedTurretYaw() {
        var encoderReading = turretMotor.getPosition();
        var timestamp = encoderReading.getTimestamp();
        if (timestamp.isValid()) {
            timestampedTurretYaw.timestamp = timestamp.getTime();
        } else {
            timestampedTurretYaw.timestamp = Timer.getTimestamp();
        }
        timestampedTurretYaw.value.mut_replace(
                encoderReading.getValueAsDouble(), turretEncoder.getInternalEncoderUnits());
        return timestampedTurretYaw;
    }

    /**
     * Get the current angular velocity of the turret about its axis of rotation.
     */
    public AngularVelocity getTurretYawRate() {
        return turretEncoder.getVelocity();
    }

    public boolean absoluteTargetIsAligned() {
        return turretAbsolutePid.atSetpoint();
    }

    public void alignAbsolute(TurretSetpoint setpoint) {
        alignAbsolute(setpoint.goalAngle(), setpoint.goalVelocity());
    }

    /**
     * Align to a specified goalAngle and goalVelocity in robot coordinates.
     */
    public void alignAbsolute(Angle goalAngle, AngularVelocity goalVelocity) {
        Angle moduloGoalAngle = angleHelper.turretAngleModulus(goalAngle);
        if (moduloGoalAngle == null) {
            // desired angle is outside the turret's range of motion
            turretMotor.stopMotor();
            return;
        }

        State currentState =
                new State(getTurretYaw().in(Units.Radians), getTurretYawRate().in(Units.RadiansPerSecond));
        State goalState = new State(moduloGoalAngle.in(Units.Radians), goalVelocity.in(Units.RadiansPerSecond));
        State setpoint = profile.calculate(Constants.LOOP_PERIOD, currentState, goalState);

        this.goalAngle.mut_replace(setpoint.position, Units.Radians);
        this.goalVelocity.mut_replace(setpoint.velocity, Units.RadiansPerSecond);

        this.turretAbsolutePid.setReference(this.goalAngle, this.goalVelocity);
    }

    public boolean relativeTargetIsVisible() {
        return targetingHelper.targetIsVisible();
    }

    public boolean relativeTargetIsAligned() {
        return targetingHelper.targetIsVisible() && turretRelativePid.atSetpoint();
    }

    public void alignRelative() {
        if (relativeTargetIsVisible() == false) {
            // No target visible
            turretMotor.stopMotor();
            return;
        }

        double result = turretRelativePid.calculate(targetingHelper.getTx(), 0);
        turretMotor.setVoltage(result);
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    @Override
    public void periodic() {
        targetingHelper.targetNearestTag(DriverStation.getAlliance().orElse(Alliance.Red));

        relativeEncoderPublisher.set(turretEncoder.getPosition().in(Units.Degrees));
        absEncoder1Publisher.set(absEncoder1.getPosition().in(Units.Rotations));
        absEncoder2Publisher.set(absEncoder2.getPosition().in(Units.Rotations));
        targetTagPublisher.set(targetingHelper.getTargetId());

        // The vernier encoder calculation is iterative, so it might be too expensive to calculate it on every loop.
        // But it's also incredibly useful information for debugging, so let's keep this line for now and remove it
        // if it becomes a problem.
        vernierEncoderPublisher.set(vernierEncoder.getPosition().in(Units.Rotations));

        var desiredNeutralMode = coastMotorEntry.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        if (desiredNeutralMode != turretMotorConfig.MotorOutput.NeutralMode) {
            turretMotorConfig.MotorOutput.NeutralMode = desiredNeutralMode;
            turretMotor.getConfigurator().apply(turretMotorConfig);
        }
    }
}
