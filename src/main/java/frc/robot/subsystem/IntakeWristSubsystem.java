package frc.robot.subsystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.MoSparkConfigurator;

public class IntakeWristSubsystem extends SubsystemBase {
    private final SparkFlex intakeWrist;

    private final MoSparkConfigurator intakeWristConfig;

    private MutCurrent intakeWristCurrent = Units.Amps.mutable(0);

    public IntakeWristSubsystem() {
        intakeWrist = new SparkFlex(Constants.INTAKE_WRIST_PORT.address(), MotorType.kBrushless);

        intakeWristConfig = MoSparkConfigurator.forSparkFlex(intakeWrist);

        intakeWristConfig.accept(config -> config.smartCurrentLimit(
                        (int) MoPrefs.intakeWristSmartCurrentLimit.get().in(Units.Amps))
                .idleMode(IdleMode.kCoast)
                .inverted(false));

        MoPrefs.intakeWristSmartCurrentLimit.subscribe(
                limit -> intakeWristConfig.accept(config -> config.smartCurrentLimit((int) limit.in(Units.Amps))));
    }

    public void wristOut() {
        intakeWrist.setVoltage(-MoPrefs.intakeWristVoltage.get().in(Units.Volts));
    }

    public void wristIn() {
        intakeWrist.setVoltage(MoPrefs.intakeWristVoltage.get().in(Units.Volts));
    }

    public void holdWristOut() {
        intakeWrist.setVoltage(-MoPrefs.intakeWristHoldVoltage.get().in(Units.Volts));
    }

    public void holdWristIn() {
        intakeWrist.setVoltage(MoPrefs.intakeWristHoldVoltage.get().in(Units.Volts));
    }

    public void stopWristMotor() {
        intakeWrist.setVoltage(0);
    }

    public Current getIntakeWristCurrent() {
        double current = intakeWrist.getOutputCurrent();
        intakeWristCurrent.mut_replace(current, Units.Amps);
        return intakeWristCurrent;
    }
}
