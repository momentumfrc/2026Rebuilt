package first.robot.subsystem;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import first.robot.Constants;
import first.robot.MoPrefs;
import first.robot.molib.NTHelpers;
import first.robot.molib.encoder.DIOAbsEncoder;
import first.robot.molib.encoder.MoRotationEncoder;
import first.robot.molib.encoder.absolute.MoAbsoluteEncoder;
import first.robot.molib.encoder.absolute.VernierEncoder;
import first.robot.molib.motune.MoTuner;
import first.robot.molib.motune.TunerUtils;
import first.robot.molib.pid.MoTalonFxProfilePID;
import first.robot.molib.prefs.MoPrefsUtils;
import first.robot.shootutils.TurretTargeting;
import first.robot.shootutils.TurretTargeting.TurretSetpoint;
import first.robot.util.LimelightTargetingHelper;
import first.robot.util.OdometryTargetingHelper;
import first.robot.util.SysIdUtil;
import first.robot.util.TurretAngleHelper;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.driverstation.Alert;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.Gamepad;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.State;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.BooleanPublisher;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.IntegerPublisher;
import org.wpilib.smartdashboard.SendableChooser;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;

public class TurretSubsystem extends SubsystemBase {
    private static final int MAIN_GEAR_TOOTH_COUNT = 85;
    private static final int ENCODER_1_GEAR_TOOTH_COUNT = 12;
    private static final int ENCODER_2_GEAR_TOOTH_COUNT = 13;

    private static final int TRAPEZOID_STATE_RESET_CUTOFF = 30;

    public static final Transform2d robotToTurret =
            new Transform2d(new Translation2d(-0.144780, -0.031750), Rotation2d.kZero);
    public static final Transform2d turretToCamera = new Transform2d(new Translation2d(0.181301, 0), Rotation2d.kZero);

    private enum TurretAlignMode {
        ODOMETRY,
        LL_CROSSHAIRS
    }

    private final SendableChooser<TurretAlignMode> alignModeChooser = NTHelpers.enumToChooser(TurretAlignMode.class);

    private final TalonFX turretMotor;
    private final TalonFXConfiguration turretMotorConfig;
    private final MoRotationEncoder turretEncoder;

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
    private final Alert encodersDisconnectedAlert = new Alert("turret absolute encoder disconnected", Alert.Level.HIGH);

    private TurretAngleHelper angleHelper;

    private TrapezoidProfile profile;
    private TrapezoidProfile.State absoluteSetpoint = new TrapezoidProfile.State();
    private final MoTalonFxProfilePID<AngleUnit, AngularVelocityUnit> turretAbsolutePid;
    private final PIDController turretRelativePid;

    private Rotation2d lastOORAngle = null;
    private final LinearFilter turretOORVelocityFilter =
            LinearFilter.movingAverage((int) (0.1 / Constants.LOOP_PERIOD));

    private final LimelightTargetingHelper targetingHelper;
    private final DoublePublisher relativeEncoderPublisher;
    private final DoublePublisher absEncoder1Publisher;
    private final DoublePublisher absEncoder2Publisher;
    private final DoublePublisher vernierEncoderPublisher;
    private final IntegerPublisher targetTagPublisher;
    private final BooleanPublisher hitForwardSoftLimit;
    private final BooleanPublisher hitReverseSoftLimit;
    private final DoublePublisher goalAnglePublisher;
    private final DoublePublisher goalVelocityPublisher;
    private final BooleanEntry targetInRange;

    private final BooleanEntry passiveTrackingEntry;
    private final BooleanEntry coastMotorEntry;
    private final BooleanEntry hasZero;
    private final BooleanEntry rezeroEveryLoop;

    public TurretSubsystem() {
        /* ==== DASHBOARD SETUP ==== */
        var table = NTHelpers.getTable("turret");
        relativeEncoderPublisher = table.getDoubleTopic("Relative Encoder").publish();
        absEncoder1Publisher = table.getDoubleTopic("Abs Encoder 1").publish();
        absEncoder2Publisher = table.getDoubleTopic("Abs Encoder 2").publish();
        vernierEncoderPublisher = table.getDoubleTopic("Vernier Encoder").publish();
        targetTagPublisher = table.getIntegerTopic("Target Tag ID").publish();
        hitForwardSoftLimit = table.getBooleanTopic("Forward Soft Limit").publish();
        hitReverseSoftLimit = table.getBooleanTopic("Reverse Soft Limit").publish();
        goalAnglePublisher = table.getDoubleTopic("Goal Angle (degs)").publish();
        goalVelocityPublisher = table.getDoubleTopic("Goal Speed (degs_s)").publish();

        targetInRange = table.getBooleanTopic("Target In Range").getEntry(false);

        passiveTrackingEntry = NTHelpers.getBooleanEntry(table, "Passive Tracking", true);
        coastMotorEntry = NTHelpers.getBooleanEntry(table, "Coast Motor", false);
        hasZero = NTHelpers.getBooleanEntry(table, "Has Zero", false);
        rezeroEveryLoop = NTHelpers.getBooleanEntry(table, "Rezero Every Loop", false);

        NTHelpers.publishSendable(table, "Align Mode", alignModeChooser);

        /* ==== MOTOR SETUP === */
        this.turretMotor = new TalonFX(Constants.TURRET_MOTOR.address(), CANBus.systemcore(Constants.DEFAULT_CAN_BUS));
        this.turretMotorConfig = new TalonFXConfiguration();
        this.turretEncoder = MoRotationEncoder.forTalonFx(turretMotor, Units.Rotations, turretMotorConfig);

        turretMotorConfig
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                        .withReverseSoftLimitThreshold(
                                MoPrefs.turretMinSoftLimit.get().in(this.turretEncoder.getInternalEncoderUnits()))
                        .withReverseSoftLimitEnable(true)
                        .withForwardSoftLimitThreshold(
                                MoPrefs.turretMaxSoftLimit.get().in(this.turretEncoder.getInternalEncoderUnits()))
                        .withForwardSoftLimitEnable(true))
                .withVoltage(new VoltageConfigs()
                        .withPeakForwardVoltage((Voltage) MoPrefs.turretMaxPower.get())
                        .withPeakReverseVoltage(
                                (Voltage) MoPrefs.turretMaxPower.get().unaryMinus()))
                .withClosedLoopRamps(
                        new ClosedLoopRampsConfigs().withVoltageClosedLoopRampPeriod(MoPrefs.turretVoltRampRate.get()))
                .withOpenLoopRamps(
                        new OpenLoopRampsConfigs().withVoltageOpenLoopRampPeriod(MoPrefs.turretVoltRampRate.get()));
        turretMotor.getConfigurator().apply(turretMotorConfig);

        MoPrefsUtils.multiSubscribe(MoPrefs.turretMinSoftLimit, MoPrefs.turretMaxSoftLimit, (min, max) -> {
            turretMotorConfig
                    .SoftwareLimitSwitch
                    .withReverseSoftLimitThreshold(min.in(turretEncoder.getInternalEncoderUnits()))
                    .withForwardSoftLimitThreshold(max.in(this.turretEncoder.getInternalEncoderUnits()));
            turretMotor.getConfigurator().apply(turretMotorConfig);
        });

        MoPrefs.turretMaxPower.subscribe(voltage -> {
            turretMotorConfig.Voltage.withPeakForwardVoltage((Voltage) voltage).withPeakReverseVoltage((Voltage)
                    voltage.unaryMinus());
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
                    hasZero.set(false);
                    zeroEncoder();
                },
                true);

        /* ==== PID SETUP ==== */
        MoPrefsUtils.multiSubscribe(
                MoPrefs.turretMaxVelocity,
                MoPrefs.turretMaxAcceleration,
                (maxVel, maxAcc) -> {
                    this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(
                            maxVel.in(Units.DegreesPerSecond), maxAcc.in(Units.DegreesPerSecondPerSecond)));
                },
                true);

        this.turretAbsolutePid = new MoTalonFxProfilePID<AngleUnit, AngularVelocityUnit>(
                turretMotor, turretEncoder.getInternalEncoderUnits(), turretMotorConfig);
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
    }

    public TurretAngleHelper getAngleHelper() {
        return angleHelper;
    }

    private boolean absEncodersAreConnected() {
        var dio1 = ((DIOAbsEncoder) absEncoder1.getMoEncoder().getEncoder()).getEncoder();
        var dio2 = ((DIOAbsEncoder) absEncoder2.getMoEncoder().getEncoder()).getEncoder();

        return dio1.isConnected() && dio2.isConnected();
    }

    public void zeroEncoder() {
        if (!absEncodersAreConnected()) {
            return;
        }

        var absolutePosition = vernierEncoder.getPosition();
        if (absolutePosition.isEmpty()) {
            return;
        }

        turretEncoder.setPosition(absolutePosition.get().plus(MoPrefs.turretRelativeEncoderOffset.get()));
        hasZero.set(true);
    }

    /**
     * Get the current angular position of the turret about its axis of rotation.
     */
    public Angle getTurretYaw() {
        return turretEncoder.getPosition();
    }

    /**
     * Get the current angular velocity of the turret about its axis of rotation.
     */
    public AngularVelocity getTurretYawRate() {
        return turretEncoder.getVelocity();
    }

    public void align(TurretSetpoint setpoint) {
        switch (alignModeChooser.getSelected()) {
            case ODOMETRY -> alignAbsolute(setpoint);
            case LL_CROSSHAIRS -> {
                if (relativeTargetIsVisible()) {
                    alignRelative();
                } else {
                    alignAbsolute(setpoint);
                }
            }
        }
    }

    public boolean targetIsAligned() {
        return switch (alignModeChooser.getSelected()) {
            case ODOMETRY -> absoluteTargetIsAligned();
            case LL_CROSSHAIRS -> relativeTargetIsAligned();
        };
    }

    public boolean absoluteTargetIsAligned() {
        return turretAbsolutePid.atSetpoint();
    }

    public void alignAbsolute(TurretSetpoint setpoint) {
        alignAbsolute(setpoint.goalAngle(), setpoint.goalVelocity());
    }

    private double calculateOutOfRangeVelocity(Rotation2d angle) {
        if (lastOORAngle == null) {
            turretOORVelocityFilter.reset();
            lastOORAngle = angle;
        }
        double velocity =
                turretOORVelocityFilter.calculate(angle.minus(lastOORAngle).getDegrees() / Constants.LOOP_PERIOD);
        lastOORAngle = angle;
        return velocity;
    }

    public boolean targetInRange() {
        return targetInRange.get();
    }

    /**
     * Align to a specified goalAngle and goalVelocity in robot coordinates.
     */
    public void alignAbsolute(Angle goalAngle, AngularVelocity goalVelocity) {
        goalAnglePublisher.set(goalAngle.in(Units.Degrees));
        goalVelocityPublisher.set(goalVelocity.in(Units.DegreesPerSecond));

        if (hasZero.get() == false) {
            stop();
            return;
        }

        TurretAngleHelper.Result result = angleHelper.turretAngleModulus(goalAngle);
        targetInRange.set(result.inRange());

        State goalState;
        if (result.inRange()) {
            goalState = new State(result.angle().getDegrees(), goalVelocity.in(Units.DegreesPerSecond));
            lastOORAngle = null;
        } else {
            goalState = new State(result.angle().getDegrees(), calculateOutOfRangeVelocity(result.angle()));
        }

        if (Math.abs(absoluteSetpoint.position - turretEncoder.getPosition().in(Units.Degrees))
                > TRAPEZOID_STATE_RESET_CUTOFF) {
            absoluteSetpoint = new State(
                    turretEncoder.getPosition().in(Units.Degrees),
                    turretEncoder.getVelocity().in(Units.DegreesPerSecond));
        }

        absoluteSetpoint = profile.calculate(Constants.LOOP_PERIOD, absoluteSetpoint, goalState);

        this.turretAbsolutePid.setReference(
                Units.Degrees.of(absoluteSetpoint.position), Units.DegreesPerSecond.of(absoluteSetpoint.velocity));
    }

    public boolean relativeTargetIsVisible() {
        return targetingHelper.targetIsVisible();
    }

    public boolean relativeTargetIsAligned() {
        return targetingHelper.targetIsVisible() && turretRelativePid.atSetpoint();
    }

    public void alignRelative() {
        if (hasZero.get() == false) {
            stop();
            return;
        }

        if (relativeTargetIsVisible() == false) {
            // No target visible
            stop();
            targetInRange.set(false);
            return;
        }

        targetInRange.set(true);

        double result = turretRelativePid.calculate(targetingHelper.getTx(), 0);
        turretMotor.setVoltage(result);
    }

    public void stop() {
        turretMotor.stopMotor();
    }

    public boolean shouldEnablePassiveTracking() {
        return passiveTrackingEntry.get();
    }

    public Command passiveTargetingCommand(TurretTargeting targeting) {
        return run(() -> {
                    if (shouldEnablePassiveTracking()) {
                        var target = OdometryTargetingHelper.getHubTarget(
                                MatchState.getAlliance().orElse(Alliance.RED));
                        var firingSolution = targeting.targetPositionStationary(target.toTranslation2d());
                        align(firingSolution);
                    } else {
                        stop();
                    }
                })
                .withName("TurretPassiveTargetingCommand");
    }

    public Command testCommand(Gamepad testController) {
        return run(() -> {
                    double x = -1 * testController.getLeftY();
                    double y = -1 * testController.getLeftX();
                    if (Math.hypot(x, y) < 0.05) {
                        stop();
                    } else {
                        double goalRadians = Math.atan2(y, x);
                        var result = angleHelper.turretAngleModulusRads(goalRadians);
                        targetInRange.set(result.inRange());

                        this.turretAbsolutePid.setReference(
                                Units.Radians.of(result.angle().getRadians()), Units.RadiansPerSecond.zero());
                    }
                })
                .withName("TurretTestCommand");

        /*return run( () -> {
            double spd = -1 * testController.getLeftY();
            var spd2 = MoPrefs.turretMaxVelocity.get().times(spd);
            this.turretAbsolutePid.setReference(getTurretYaw(), spd2);
        }).withName("TurretTestCommand");
        */
    }

    @Override
    public void periodic() {
        targetingHelper.targetNearestTag(MatchState.getAlliance().orElse(Alliance.RED));

        relativeEncoderPublisher.set(turretEncoder.getPosition().in(Units.Degrees));
        absEncoder1Publisher.set(absEncoder1.getPosition().in(Units.Rotations));
        absEncoder2Publisher.set(absEncoder2.getPosition().in(Units.Rotations));
        targetTagPublisher.set(targetingHelper.getTargetId());

        hitForwardSoftLimit.set(turretMotor.getFault_ForwardSoftLimit().getValue());
        hitReverseSoftLimit.set(turretMotor.getFault_ReverseSoftLimit().getValue());

        encodersDisconnectedAlert.set(absEncodersAreConnected() == false);

        // The vernier encoder calculation is iterative, so it might be too expensive to calculate it on every loop.
        // But it's also incredibly useful information for debugging, so let's keep this line for now and remove it
        // if it becomes a problem.
        vernierEncoderPublisher.set(vernierEncoder
                .getPosition()
                .map(angle -> angle.in(Units.Rotations))
                .orElse(Double.NaN));

        if (hasZero.get() == false || rezeroEveryLoop.get()) {
            zeroEncoder();
        }

        var desiredNeutralMode = coastMotorEntry.get() ? NeutralModeValue.Coast : NeutralModeValue.Brake;
        if (desiredNeutralMode != turretMotorConfig.MotorOutput.NeutralMode) {
            turretMotorConfig.MotorOutput.NeutralMode = desiredNeutralMode;
            turretMotor.getConfigurator().refresh(turretMotorConfig.Feedback);
            turretMotor.getConfigurator().apply(turretMotorConfig);
        }
    }

    public SysIdRoutine.Mechanism getSysIdMechanism() {
        return SysIdUtil.sysIdMechanismForTalonFx(this, "turret", turretMotor, turretEncoder);
    }
}
