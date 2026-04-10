package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.IntakeRollerSubsystem;

public class RollerCommands {
    public static Command idleIntakeRollerCommand(IntakeRollerSubsystem roller) {
        return roller.run(roller::stopRollerMotor).withName("IdleIntakeRollerCommand");
    }

    public static Command intakeRollerDefaultCommand(IntakeRollerSubsystem roller) {
        return idleIntakeRollerCommand(roller);
    }

    public static Command runIntakeRollerCommand(IntakeRollerSubsystem roller) {
        return roller.run(roller::rollerIntake).withName("RunIntakeRollersCommand");
    }

    public static Command runIntakeRollerReverseCommand(IntakeRollerSubsystem roller) {
        return roller.run(roller::rollerExtake).withName("RunIntakeRollersReverseCommand");
    }
}
