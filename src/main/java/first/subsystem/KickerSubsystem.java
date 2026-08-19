package frc.robot.subsystem;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.MoSparkConfigurator;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoSparkMaxPID;

public class KickerSubsystem extends SubsystemBase {

    private final SparkFlex motor;
    private final MoSparkConfigurator config;

    private final MoRotationEncoder encoder;

    private final MoSparkMaxPID<AngleUnit, AngularVelocityUnit> pid;

    public KickerSubsystem() {
        motor = new SparkFlex(Constants.KICKER_PORT.address(), MotorType.kBrushless);
        config = MoSparkConfigurator.forSparkFlex(motor);

        config.accept(config -> config.smartCurrentLimit(
                        (int) MoPrefs.kickerCurrentLimit.get().in(Units.Amps))
                .inverted(true)
                .idleMode(IdleMode.kCoast));

        MoPrefs.kickerCurrentLimit.subscribe(
                limit -> config.accept(config -> config.smartCurrentLimit((int) limit.in(Units.Amps))));

        encoder = MoRotationEncoder.forSparkRelative(motor, Units.Revolutions);
        MoPrefs.kickerEncoderScale.subscribe(encoder::setConversionFactor, true);

        pid = new MoSparkMaxPID<>(MoSparkMaxPID.Type.VELOCITY, motor, ClosedLoopSlot.kSlot0, encoder, config);

        TunerUtils.forMoSparkMax(pid, "Kicker PID");
    }

    public void runAtSpeed(AngularVelocity speed) {
        pid.setVelocityReference(speed);
    }

    public void run() {
        pid.setVelocityReference(MoPrefs.kickerRunSpeed.get());
    }

    public void stop() {
        pid.setVelocityReference(Units.RevolutionsPerSecond.zero());
    }

    public SysIdRoutine.Mechanism getSysIdMechanism() {
        return new SysIdRoutine.Mechanism(
                motor::setVoltage,
                log -> log.motor("kicker")
                        .voltage(Units.Volts.of(motor.getBusVoltage() * motor.getAppliedOutput()))
                        .value(
                                "position",
                                encoder.getPositionInEncoderUnits(),
                                encoder.getInternalEncoderUnits().name())
                        .value(
                                "velocity",
                                encoder.getVelocityInEncoderUnits(),
                                encoder.getInternalEncoderVelocityUnits().name()),
                this);
    }
}
