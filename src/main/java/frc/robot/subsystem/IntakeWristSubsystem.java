package frc.robot.subsystem;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.MoSparkConfigurator;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.MoTuner;
import frc.robot.molib.pid.MoSparkMaxPID;
import frc.robot.util.NTHelpers;

public class IntakeWristSubsystem extends SubsystemBase {
    private static final double POSITION_TOLERANCE = 0.05;

    private final SparkFlex intakeWrist;

    // Note: the built-in Spark kCos arm feedforward assumes the internal encoder is setup
    // so that the zero position is perfectly horizontal
    private final MoRotationEncoder wristEncoder;
    private final MoSparkMaxPID<AngleUnit, AngularVelocityUnit> wristPid;

    private final MoSparkConfigurator intakeWristConfig;

    private MutCurrent intakeWristCurrent = Units.Amps.mutable(0);

    private final DoublePublisher positionPublisher;
    private final DoublePublisher wristCurrentPublisher;
    private final DoublePublisher wristVoltagePublisher;

    private final BooleanEntry hasZeroEntry;
    private final BooleanEntry coastMotorEntry;
    private IdleMode currentIdleMode;

    public IntakeWristSubsystem() {
        intakeWrist = new SparkFlex(Constants.INTAKE_WRIST_PORT.address(), MotorType.kBrushless);
        intakeWristConfig = MoSparkConfigurator.forSparkFlex(intakeWrist);
        intakeWristConfig.accept(config -> config.smartCurrentLimit(
                        (int) MoPrefs.intakeWristSmartCurrentLimit.get().in(Units.Amps))
                .openLoopRampRate(MoPrefs.intakeRampTime.get().in(Units.Seconds))
                .closedLoopRampRate(MoPrefs.intakeRampTime.get().in(Units.Seconds))
                .idleMode(IdleMode.kBrake)
                .inverted(false));
        currentIdleMode = IdleMode.kBrake;

        MoPrefs.intakeWristSmartCurrentLimit.subscribe(
                limit -> intakeWristConfig.accept(config -> config.smartCurrentLimit((int) limit.in(Units.Amps))));

        MoPrefs.intakeRampTime.subscribe(rampTime -> intakeWristConfig.accept(config ->
                config.openLoopRampRate(rampTime.in(Units.Seconds)).closedLoopRampRate(rampTime.in(Units.Seconds))));

        wristEncoder = MoRotationEncoder.forSparkRelative(intakeWrist, Units.Rotations);
        wristPid = new MoSparkMaxPID<>(
                MoSparkMaxPID.Type.POSITION, intakeWrist, ClosedLoopSlot.kSlot0, wristEncoder, intakeWristConfig);

        MoTuner.builder("Intake Wrist")
                .pid(wristPid)
                .motorFF(wristPid)
                .iZone(wristPid::setIZone)
                .setpoint(wristPid::getSetpoint)
                .parameter(
                        "kG",
                        kG -> wristPid.setConfigOption((config, slot) -> config.closedLoop.feedForward.kCos(kG, slot)))
                .measurement(wristPid::getLastMeasurement)
                .onPopulateFinished(wristPid)
                .safeBuild();

        var table = NTHelpers.getTable("Intake Wrist");
        wristCurrentPublisher = table.getDoubleTopic("Intake Wrist Current").publish();
        wristVoltagePublisher = table.getDoubleTopic("Intake Wrist Voltage").publish();
        positionPublisher =
                table.getDoubleTopic("Intake Wrist Position (rotations)").publish();
        hasZeroEntry = NTHelpers.getBooleanEntry(table, "Has Zero", false);
        coastMotorEntry = NTHelpers.getBooleanEntry(table, "Coast Motor", false);
    }

    public void moveVoltage(Voltage voltage) {
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

    public void zeroEncoder() {
        wristEncoder.setPosition(MoPrefs.intakeWristZeroPosition.get());
        hasZeroEntry.set(true);
    }

    @Override
    public void periodic() {
        wristCurrentPublisher.set(intakeWristCurrent.in(Units.Amps));
        positionPublisher.set(intakeWrist.getEncoder().getPosition());

        wristVoltagePublisher.set(intakeWrist.getAppliedOutput() * intakeWrist.getBusVoltage());

        var desiredMotorIdleMode = coastMotorEntry.get() ? IdleMode.kCoast : IdleMode.kBrake;
        if (desiredMotorIdleMode != currentIdleMode) {
            intakeWristConfig.accept(config -> config.idleMode(desiredMotorIdleMode));
            currentIdleMode = desiredMotorIdleMode;
        }
    }
}
