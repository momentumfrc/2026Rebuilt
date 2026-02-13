package frc.robot.subsystem;

import static edu.wpi.first.math.util.Units.degreesToRadians;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.encoder.absolute.MoAbsoluteEncoder;
import frc.robot.molib.encoder.absolute.VernierEncoder;
import frc.robot.molib.motune.MoTuner;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxPID;
import frc.robot.molib.prefs.MoPrefsUtils;
import frc.robot.util.LimelightTargetingHelper;
import frc.robot.util.NTHelpers;
import frc.robot.util.NTHelpers.BooleanChangeSubscriber;
import frc.robot.util.TurretAngleHelper;

public class TurretSubsystem extends SubsystemBase {
    private static final int MAIN_GEAR_TOOTH_COUNT = 85;
    private static final int ENCODER_1_GEAR_TOOTH_COUNT = 15;
    private static final int ENCODER_2_GEAR_TOOTH_COUNT = 16;

    private static final Translation3d turretPositionInRobotCoordinates =
            new Translation3d(-0.154305, -0.031750, 0.381);
    private static final Transform3d limelightPositionRelativeToTurret =
            new Transform3d(0.181656, 0, 0.146352, new Rotation3d(0, degreesToRadians(15), 0));

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
    private TurretAngleHelper angleHelper;

    private final MoTalonFxPID<AngleUnit, AngularVelocityUnit> turretAbsolutePid;
    private final PIDController turretRelativePid;

    private final LimelightTargetingHelper targetingHelper;

    private final DoublePublisher relativeEncoderPublisher;
    private final DoublePublisher absEncoder1Publisher;
    private final DoublePublisher absEncoder2Publisher;
    private final DoublePublisher vernierEncoderPublisher;
    private final IntegerPublisher targetTagPublisher;

    private final BooleanChangeSubscriber coastMotorSubscriber;

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
                        .withForwardSoftLimitEnable(true));
        turretMotor.getConfigurator().apply(turretMotorConfig);

        MoPrefsUtils.multiSubscribe(MoPrefs.turretMinSoftLimit, MoPrefs.turretMaxSoftLimit, (min, max) -> {
            turretMotorConfig
                    .SoftwareLimitSwitch
                    .withReverseSoftLimitThreshold((Angle) min)
                    .withForwardSoftLimitThreshold((Angle) max);
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

        this.turretEncoder = MoRotationEncoder.forTalonFx(turretMotor, Units.Radians);
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
        this.turretAbsolutePid = new MoTalonFxPID<AngleUnit, AngularVelocityUnit>(
                MoTalonFxPID.Type.POSITION, turretMotor, turretEncoder.getInternalEncoderUnits());
        TunerUtils.forMoTalonFx(turretAbsolutePid, "Turret Absolute Position");

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

        coastMotorSubscriber = NTHelpers.getBooleanChangeSubscriber(table, "Coast Motor", false);
    }

    /**
     * Get the current angular position of the turret about its axis of rotation.
     */
    public Angle getTurretYaw() {
        return turretEncoder.getPosition();
    }

    /**
     * Get the current pose of the turret in robot-relative coordinates.
     */
    public Pose3d getTurretPose() {
        return new Pose3d(
                turretPositionInRobotCoordinates,
                new Rotation3d(0, 0, getTurretYaw().in(Units.Radians)));
    }

    /**
     * Get the current pose of the targeting limelight attached to the turret in robot-relative coordinates.
     */
    public Pose3d getTurretLimelightPose() {
        return getTurretPose().plus(limelightPositionRelativeToTurret);
    }

    public void alignAbsolute(Angle angle) {
        turretAbsolutePid.setPositionReference(angleHelper.turretAngleModulus(angle));
    }

    public void alignRelative() {
        targetingHelper.targetNearestTag(DriverStation.getAlliance().orElse(Alliance.Red));
        if (targetingHelper.targetIsVisible() == false) {
            // No target visible
            turretMotor.stopMotor();
            return;
        }

        double result = turretRelativePid.calculate(targetingHelper.getTx(), 0);
        turretMotor.set(result);
    }

    @Override
    public void periodic() {
        relativeEncoderPublisher.set(turretEncoder.getPosition().in(Units.Degrees));
        absEncoder1Publisher.set(absEncoder1.getPosition().in(Units.Rotations));
        absEncoder2Publisher.set(absEncoder2.getPosition().in(Units.Rotations));
        targetTagPublisher.set(targetingHelper.getTargetId());

        // The vernier encoder calculation is iterative, so it might be too expensive to calculate it on every loop.
        // But it's also incredibly useful information for debugging, so let's keep this line for now and remove it
        // if it becomes a problem.
        vernierEncoderPublisher.set(vernierEncoder.getPosition().in(Units.Rotations));

        var coastMotor = coastMotorSubscriber.get();
        if (coastMotor != BooleanChangeSubscriber.Value.NO_CHANGE) {
            turretMotorConfig.MotorOutput.NeutralMode = (coastMotor == BooleanChangeSubscriber.Value.TRUE)
                    ? NeutralModeValue.Coast
                    : NeutralModeValue.Brake;
            turretMotor.getConfigurator().apply(turretMotorConfig);
        }
    }
}
