package first.commands.intake;

import first.subsystem.IntakeRollerSubsystem;
import org.wpilib.command2.Command;

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
