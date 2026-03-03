package frc.robot.subsystem;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.MoSparkConfigurator;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoSparkMaxPID;
import frc.robot.util.NTHelpers;

@Logged
public class IndexerSubsystem extends SubsystemBase {

    private final SparkFlex motor;
    private final MoSparkConfigurator config;

    private final MoRotationEncoder encoder;

    private final MoSparkMaxPID<AngleUnit, AngularVelocityUnit> pid;

    private final DoublePublisher speedPublisher;

    public IndexerSubsystem() {
        motor = new SparkFlex(Constants.INDEXER_PORT.address(), MotorType.kBrushless);
        config = MoSparkConfigurator.forSparkFlex(motor);

        config.accept(config -> config.smartCurrentLimit(
                        (int) MoPrefs.indexerRollerSmartCurrentLimit.get().in(Units.Amps))
                .inverted(false)
                .idleMode(IdleMode.kCoast));
        MoPrefs.indexerRollerSmartCurrentLimit.subscribe(
                limit -> config.accept(config -> config.smartCurrentLimit((int) limit.in(Units.Amps))));

        encoder = MoRotationEncoder.forSparkRelative(motor, Units.Revolutions);
        MoPrefs.indexerEncoderScale.subscribe(encoder::setConversionFactor, true);

        pid = new MoSparkMaxPID<>(MoSparkMaxPID.Type.VELOCITY, motor, ClosedLoopSlot.kSlot0, encoder, config);

        TunerUtils.forMoSparkMax(pid, "Indexer PID");

        NetworkTable table = NTHelpers.getTable("indexer");

        speedPublisher = table.getDoubleTopic("Indexer Speed").publish();
    }

    public void run() {
        pid.setVelocityReference(MoPrefs.indexerRunSpeed.get());
    }

    public void stop() {
        pid.setVelocityReference(Units.RevolutionsPerSecond.zero());
    }

    public SysIdRoutine.Mechanism getSysIdMechanism() {
        return new SysIdRoutine.Mechanism(
                motor::setVoltage,
                log -> log.motor("indexer")
                        .voltage(Units.Volts.of(motor.getBusVoltage() * motor.get()))
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

    @Override
    public void periodic() {
        speedPublisher.set(encoder.getVelocity().in(Units.RPM));
    }
}
