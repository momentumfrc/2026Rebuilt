package first.subsystem;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import first.Constants;
import first.MoPrefs;
import first.molib.MoSparkConfigurator;
import first.molib.MoUnits;
import first.molib.NTHelpers;
import first.molib.encoder.MoRotationEncoder;
import first.molib.motune.MoTuner;
import first.molib.pid.MoSparkMaxArmProfilePID;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.driverstation.XboxController;
import org.wpilib.math.util.MathUtil;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class IntakeWristSubsystem extends SubsystemBase {
    private static final double POSITION_TOLERANCE = 0.05;

    private final SparkFlex intakeWrist;

    private final MoRotationEncoder wristEncoder;
    private final MoSparkMaxArmProfilePID wristPid;

    private final MoSparkConfigurator intakeWristConfig;

    private Current intakeWristCurrent = Units.Amps.of(0);

    private final DoublePublisher positionPublisher;
    private final DoublePublisher wristCurrentPublisher;
    private final DoublePublisher wristVoltagePublisher;

    private final BooleanEntry hasZeroEntry;

    public IntakeWristSubsystem() {
        //TODO: Edit busID!!!
        intakeWrist = new SparkFlex(0, Constants.INTAKE_WRIST_PORT.address(), MotorType.kBrushless);
        intakeWristConfig = MoSparkConfigurator.forSparkFlex(intakeWrist);
        intakeWristConfig.accept(config -> config.smartCurrentLimit(
                        (int) MoPrefs.intakeWristSmartCurrentLimit.get().in(Units.Amps))
                .openLoopRampRate(MoPrefs.intakeRampTime.get().in(Units.Seconds))
                .closedLoopRampRate(MoPrefs.intakeRampTime.get().in(Units.Seconds))
                .idleMode(IdleMode.kCoast)
                .inverted(true));

        MoPrefs.intakeWristSmartCurrentLimit.subscribe(
                limit -> intakeWristConfig.accept(config -> config.smartCurrentLimit((int) limit.in(Units.Amps))));

        MoPrefs.intakeRampTime.subscribe(rampTime -> intakeWristConfig.accept(config ->
                config.openLoopRampRate(rampTime.in(Units.Seconds)).closedLoopRampRate(rampTime.in(Units.Seconds))));

        wristEncoder = MoRotationEncoder.forSparkRelative(intakeWrist, Units.Rotations);
        wristEncoder.setConversionFactor(MoUnits.EncoderTicksPerRotation.ofNative(72));

        wristPid = new MoSparkMaxArmProfilePID(intakeWrist, ClosedLoopSlot.kSlot0, wristEncoder, intakeWristConfig);

        MoTuner.builder("Intake Wrist")
                .pid(wristPid)
                .motorFF(wristPid)
                .iZone(wristPid::setIZone)
                .setpoint(wristPid::getSetpoint)
                .parameter("kG", wristPid::setG)
                .parameter("horizontalOffset", wristPid::setHorizontalOffset)
                .measurement(wristPid::getLastMeasurement)
                .stateVariable("lastFF", wristPid::getLastFF)
                .onPopulateFinished(wristPid)
                .safeBuild();

        MoPrefs.intakeWristMaxVelocity.subscribe(velocity -> wristPid.setMaxVelocity((AngularVelocity) velocity), true);
        MoPrefs.intakeWristMaxAccel.subscribe(accel -> wristPid.setMaxAcceleration((AngularAcceleration) accel), true);

        var table = NTHelpers.getTable("Intake Wrist");
        wristCurrentPublisher = table.getDoubleTopic("Intake Wrist Current").publish();
        wristVoltagePublisher = table.getDoubleTopic("Intake Wrist Voltage").publish();
        positionPublisher =
                table.getDoubleTopic("Intake Wrist Position (rotations)").publish();
        hasZeroEntry = NTHelpers.getBooleanEntry(table, "Has Zero", false);
    }

    public void moveVoltage(Voltage voltage) {
        double ff = 0;
        if (hasZeroEntry.get()) {
            ff = wristPid.calculateFF(Units.RadiansPerSecond.zero());
        }
        intakeWrist.setVoltage(voltage.in(Units.Volts) + ff);
    }

    public void moveVoltageNoFF(Voltage voltage) {
        intakeWrist.setVoltage(voltage.in(Units.Volts));
    }

    public void movePosition(Angle position) {
        if (hasZeroEntry.get() == false) {
            stopWristMotor();
            return;
        }

        wristPid.setPositionReference(position);
    }

    public void stopWristMotor() {
        intakeWrist.setVoltage(0);
    }

    public Current getIntakeWristCurrent() {
        double current = intakeWrist.getOutputCurrent().get();
        intakeWristCurrent.mut_replace(current, Units.Amps);
        return intakeWristCurrent;
    }

    public boolean atPosition(Angle position) {
        if (hasZeroEntry.get() == false) {
            return false;
        }

        return wristEncoder.getPosition().isNear(position, POSITION_TOLERANCE);
    }

    public Angle getPosition() {
        return wristEncoder.getPosition();
    }

    public void zeroEncoder() {
        if (hasZeroEntry.get()) {
            return;
        }
        wristEncoder.setPosition(Units.Rotations.zero());
        hasZeroEntry.set(true);
    }

    public boolean hasZero() {
        return hasZeroEntry.get();
    }

    public Command testCommand(XboxController testController) {
        return run(() -> {
            double y = -1 * testController.getRightY();
            double setpointRotations = MathUtil.interpolate(
                    MoPrefs.intakeWristRetractPosition.get().in(Units.Rotations),
                    MoPrefs.intakeWristDeployPosition.get().in(Units.Rotations),
                    MathUtil.inverseInterpolate(-1, 1, y));
            wristPid.setUnprofiledPositionReference(Units.Rotations.of(setpointRotations));
        });
    }

    @Override
    public void periodic() {
        wristCurrentPublisher.set(getIntakeWristCurrent().in(Units.Amps));
        positionPublisher.set(intakeWrist.getEncoder().getPosition().get());

        wristVoltagePublisher.set(intakeWrist.getAppliedOutput().get() * intakeWrist.getBusVoltage().get());
    }

    public SysIdRoutine.Mechanism getSysIdMechanism() {
        // Note: sysid usually logs velocity in RPS, but here we're logging it in RPM (we do this because the
        // feedforward calculations running on the spark use RPM, so we need kV in v/rpm). Hopefully it's fine,
        // but if sysid complains try logging the value in RPS and just multiplying the kV constant by 60.
        return new SysIdRoutine.Mechanism(
                intakeWrist::setVoltage,
                log -> log.motor("intake wrist")
                        .voltage(Units.Volts.of(intakeWrist.getBusVoltage().get() * intakeWrist.getAppliedOutput().get()))
                        .value(
                                "position",
                                wristEncoder.getPositionInEncoderUnits(),
                                wristEncoder.getInternalEncoderUnits().name())
                        .value(
                                "velocity",
                                wristEncoder.getVelocityInEncoderUnits(),
                                wristEncoder.getInternalEncoderVelocityUnits().name()),
                this);
    }
}
