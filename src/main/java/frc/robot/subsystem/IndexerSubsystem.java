package frc.robot.subsystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.MoSparkConfigurator;
import frc.robot.molib.NTHelpers;
import frc.robot.molib.encoder.MoRotationEncoder;

public class IndexerSubsystem extends SubsystemBase {

    private final SparkFlex motor;
    private final MoSparkConfigurator config;

    private final MoRotationEncoder encoder;

    private final DoublePublisher indexerEncoderPosition;
    private final DoublePublisher indexerEncoderSpeed;

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

        var table = NTHelpers.getTable("Indexer");

        indexerEncoderPosition =
                table.getDoubleTopic("Indexer Encoder Position").publish();
        indexerEncoderSpeed = table.getDoubleTopic("Indexer Encoder Speed").publish();
    }

    public void run() {
        motor.setVoltage(MoPrefs.indexerRunPower.get().in(Units.Volts));
    }

    public void stop() {
        motor.stopMotor();
    }

    public void runReverse() {
        motor.setVoltage(-1 * MoPrefs.indexerRunPower.get().in(Units.Volts));
    }

    @Override
    public void periodic() {
        indexerEncoderPosition.set(encoder.getPosition().in(Units.Revolutions));
        indexerEncoderSpeed.set(encoder.getVelocity().in(Units.RevolutionsPerSecond));
    }
}
