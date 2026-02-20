package frc.robot.subsystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.MoSparkConfigurator;

public class IntakeRollerSubsystem extends SubsystemBase {
    private final SparkFlex intakeRoller;

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
}
