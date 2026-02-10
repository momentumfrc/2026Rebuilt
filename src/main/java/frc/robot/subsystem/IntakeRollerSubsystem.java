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
        intakeRollerConfig.accept(config -> config.inverted(false).idleMode(IdleMode.kCoast));
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
