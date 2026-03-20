package frc.robot.subsystem;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.MoSparkConfigurator;
import frc.robot.molib.MoUnits;
import frc.robot.molib.NTHelpers;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.MoTuner;
import frc.robot.molib.pid.MoSparkMaxArmProfilePID;

public class IntakeWristSubsystem extends SubsystemBase {
    private static final double POSITION_TOLERANCE = 0.05;

    private final SparkFlex intakeWrist;

    private final MoRotationEncoder wristEncoder;
    private final MoSparkMaxArmProfilePID wristPid;

    private final MoSparkConfigurator intakeWristConfig;

    private MutCurrent intakeWristCurrent = Units.Amps.mutable(0);

    private final DoublePublisher positionPublisher;
    private final DoublePublisher wristCurrentPublisher;
    private final DoublePublisher wristVoltagePublisher;

    private final BooleanEntry hasZeroEntry;

    public IntakeWristSubsystem() {
        intakeWrist = new SparkFlex(Constants.INTAKE_WRIST_PORT.address(), MotorType.kBrushless);
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

        MoPrefs.intakeWristMaxVelocity.subscribe(velocity -> wristPid.setMaxVelocity((AngularVelocity) velocity));
        MoPrefs.intakeWristMaxAccel.subscribe(accel -> wristPid.setMaxAcceleration((AngularAcceleration) accel));

        var table = NTHelpers.getTable("Intake Wrist");
        wristCurrentPublisher = table.getDoubleTopic("Intake Wrist Current").publish();
        wristVoltagePublisher = table.getDoubleTopic("Intake Wrist Voltage").publish();
        positionPublisher =
                table.getDoubleTopic("Intake Wrist Position (rotations)").publish();
        hasZeroEntry = NTHelpers.getBooleanEntry(table, "Has Zero", false);
    }

    public void moveVoltage(Voltage voltage) {
        double ff = wristPid.calculateFF(Units.RadiansPerSecond.zero());
        intakeWrist.setVoltage(voltage.in(Units.Volts) + ff);
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
        double current = intakeWrist.getOutputCurrent();
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
        wristEncoder.setPosition(Units.Rotations.zero());
        hasZeroEntry.set(true);
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
        positionPublisher.set(intakeWrist.getEncoder().getPosition());

        wristVoltagePublisher.set(intakeWrist.getAppliedOutput() * intakeWrist.getBusVoltage());
    }

    public SysIdRoutine.Mechanism getSysIdMechanism() {
        // Note: sysid usually logs velocity in RPS, but here we're logging it in RPM (we do this because the
        // feedforward calculations running on the spark use RPM, so we need kV in v/rpm). Hopefully it's fine,
        // but if sysid complains try logging the value in RPS and just multiplying the kV constant by 60.
        return new SysIdRoutine.Mechanism(
                intakeWrist::setVoltage,
                log -> log.motor("intake wrist")
                        .voltage(Units.Volts.of(intakeWrist.getBusVoltage() * intakeWrist.getAppliedOutput()))
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
