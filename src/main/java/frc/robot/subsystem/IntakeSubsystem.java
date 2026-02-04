package frc.robot.subsystem;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;

public class IntakeSubsystem extends SubsystemBase{
    private final SparkFlex intakeRoller;
    private final SparkFlex intakeWrist;

    public IntakeSubsystem() {
        intakeRoller = new SparkFlex(Constants.INTAKE_ROLLER_PORT.address(), MotorType.kBrushless);
        intakeWrist = new SparkFlex(Constants.INTAKE_WRIST_PORT.address(), MotorType.kBrushless);
    }

        public void rollerOut() {
        intakeRoller.setVoltage(-MoPrefs.intakeRollerVoltage.get().in(Units.Volts));
    }

    public void rollerIn() {
        intakeRoller.setVoltage(MoPrefs.intakeRollerVoltage.get().in(Units.Volts));
    }

    public void stopRollerMotor() {
        intakeRoller.setVoltage(0);
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

}
