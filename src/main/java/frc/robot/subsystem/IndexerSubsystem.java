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

    private final SparkFlex indexerMotor;
    private final SparkFlex centeringMotor;

    private final MoSparkConfigurator indexerConfig;
    private final MoSparkConfigurator centeringConfig;

    private final MoRotationEncoder indexerEncoder;
    private final MoRotationEncoder centeringEncoder;

    private final DoublePublisher indexerEncoderPosition;
    private final DoublePublisher indexerEncoderSpeed;

    private final DoublePublisher centeringEncoderPostion;
    private final DoublePublisher centeringEncoderSpeed;

    public IndexerSubsystem() {
        indexerMotor = new SparkFlex(Constants.INDEXER_PORT.address(), MotorType.kBrushless);
        centeringMotor = new SparkFlex(Constants.CENTERING_PORT.address(), MotorType.kBrushless);

        indexerConfig = MoSparkConfigurator.forSparkFlex(indexerMotor);
        centeringConfig = MoSparkConfigurator.forSparkFlex(centeringMotor);

        indexerConfig.accept(config -> config.smartCurrentLimit(
                        (int) MoPrefs.indexerRollerSmartCurrentLimit.get().in(Units.Amps))
                .inverted(false)
                .idleMode(IdleMode.kCoast));
        MoPrefs.indexerRollerSmartCurrentLimit.subscribe(
                limit -> indexerConfig.accept(config -> config.smartCurrentLimit((int) limit.in(Units.Amps))));

        centeringConfig.accept(config -> config.smartCurrentLimit(
                        (int) MoPrefs.CenteringSmartCurrentLimit.get().in(Units.Amps))
                .inverted(true)
                .idleMode(IdleMode.kCoast));
        MoPrefs.CenteringSmartCurrentLimit.subscribe(
                limit -> centeringConfig.accept(config -> config.smartCurrentLimit((int) limit.in(Units.Amps))));

        indexerEncoder = MoRotationEncoder.forSparkRelative(indexerMotor, Units.Revolutions);
        MoPrefs.indexerEncoderScale.subscribe(indexerEncoder::setConversionFactor, true);

        centeringEncoder = MoRotationEncoder.forSparkRelative(centeringMotor, Units.Revolutions);
        MoPrefs.centeringEncoderScale.subscribe(centeringEncoder::setConversionFactor, true);

        var table = NTHelpers.getTable("Indexer");

        indexerEncoderPosition =
                table.getDoubleTopic("Indexer Encoder Position").publish();
        indexerEncoderSpeed = table.getDoubleTopic("Indexer Encoder Speed").publish();

        centeringEncoderPostion =
                table.getDoubleTopic("Centering Encoder Position").publish();
        centeringEncoderSpeed = table.getDoubleTopic("Centering Encoder Speed").publish();
    }

    public void run() {
        indexerMotor.setVoltage(MoPrefs.indexerRunPower.get().in(Units.Volts));
        centeringMotor.setVoltage(MoPrefs.centeringRunPower.get().in(Units.Volts));
    }

    public void runIndexerNoCentering() {
        indexerMotor.setVoltage(MoPrefs.indexerRunPower.get().in(Units.Volts));
        centeringMotor.stopMotor();
    }

    public void stop() {
        indexerMotor.stopMotor();
        centeringMotor.stopMotor();
    }

    public void runReverse() {
        indexerMotor.setVoltage(-1 * MoPrefs.indexerRunPower.get().in(Units.Volts));
        centeringMotor.setVoltage(-1 * MoPrefs.centeringRunPower.get().in(Units.Volts));
    }

    @Override
    public void periodic() {
        indexerEncoderPosition.set(indexerEncoder.getPosition().in(Units.Revolutions));
        indexerEncoderSpeed.set(indexerEncoder.getVelocity().in(Units.RevolutionsPerSecond));

        centeringEncoderPostion.set(centeringEncoder.getPosition().in(Units.Revolutions));
        centeringEncoderSpeed.set(centeringEncoder.getVelocity().in(Units.RevolutionsPerSecond));
    }
}
