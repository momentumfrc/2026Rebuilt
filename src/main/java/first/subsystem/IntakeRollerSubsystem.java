package first.subsystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import first.Constants;
import first.MoPrefs;
import first.molib.MoSparkConfigurator;
import first.molib.NTHelpers;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.units.Units;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final SparkFlex intakeRoller;

    private final DoublePublisher speedPublisher;
    private final MoSparkConfigurator intakeRollerConfig;

    public IntakeRollerSubsystem() {
        intakeRoller = new SparkFlex(Constants.INTAKE_ROLLER_PORT.address(), MotorType.kBrushless);
        intakeRollerConfig = MoSparkConfigurator.forSparkFlex(intakeRoller);
        intakeRollerConfig.accept(config -> config.smartCurrentLimit(
                        (int) MoPrefs.intakeRollerSmartCurrentLimit.get().in(Units.Amps))
                .inverted(false)
                .idleMode(IdleMode.kCoast));

        MoPrefs.intakeRollerSmartCurrentLimit.subscribe(
                limit -> intakeRollerConfig.accept(config -> config.smartCurrentLimit((int) limit.in(Units.Amps))));

        NetworkTable table = NTHelpers.getTable("intake");

        speedPublisher = table.getDoubleTopic("Intake Roller Speed (RPM)").publish();
    }

    public void rollerExtake() {
        intakeRoller.setVoltage(-MoPrefs.intakeRollerVoltage.get().in(Units.Volts));
    }

    public void rollerIntake() {
        intakeRoller.setVoltage(MoPrefs.intakeRollerVoltage.get().in(Units.Volts));
    }

    public void stopRollerMotor() {
        intakeRoller.setVoltage(0);
    }

    @Override
    public void periodic() {
        speedPublisher.set(intakeRoller.getEncoder().getVelocity());
    }
}
