package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystem.IntakeRollerSubsystem;

public class RollerCommands {
    public static Command idleIntakeRollerCommand(IntakeRollerSubsystem roller) {
        return Commands.run(roller::stopRollerMotor, roller).withName("IdleIntakeRollerCommand");
    }

    public static Command intakeRollerDefaultCommand(IntakeRollerSubsystem roller) {
        return idleIntakeRollerCommand(roller);
    }

    public static Command runIntakeRollerCommand(IntakeRollerSubsystem roller) {
        return Commands.run(roller::rollerIntake, roller).withName("RunIntakeRollersCommand");
    }

    public static Command runIntakeRollerReverseCommand(IntakeRollerSubsystem roller) {
        return Commands.run(roller::rollerExtake, roller).withName("RunIntakeRollersReverseCommand");
    }
}
