package frc.robot.commands.intake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MoPrefs;
import frc.robot.subsystem.IntakeWristSubsystem;

public class WristCommands {
    public enum Direction {
        IN,
        OUT;
    }

    public static Command holdIntakeWristCommand(IntakeWristSubsystem wrist, Direction direction) {
        return switch (direction) {
            case IN ->
                wrist.run(() -> wrist.moveVoltage(
                        (Voltage) MoPrefs.intakeWristHoldVoltage.get().unaryMinus()));
            case OUT -> wrist.run(() -> wrist.moveVoltage((Voltage) MoPrefs.intakeWristHoldVoltage.get()));
        };
    }

    public static Command deployIntakeWristCommand(IntakeWristSubsystem wrist) {
        return new MoveIntakeWristCommand(wrist, Direction.OUT)
                .andThen(holdIntakeWristCommand(wrist, Direction.OUT))
                .withName("DeployIntakeCommand");
    }

    public static Command retractIntakeWristCommand(IntakeWristSubsystem wrist) {
        return new MoveIntakeWristCommand(wrist, Direction.IN)
                .andThen(wrist.runOnce(wrist::zeroEncoder))
                .andThen(holdIntakeWristCommand(wrist, Direction.IN))
                .withName("RetractIntakeCommand");
    }

    public static Command intakeWristDefaultCommand(IntakeWristSubsystem wrist) {
        return retractIntakeWristCommand(wrist).withName("DefaultRetractIntakeCommand");
    }

    public static Command agitatingCommand(IntakeWristSubsystem wrist) {
        return new MoveIntakeWristToPositionCommand(wrist, MoPrefs.intakeAgitateLowPos::get)
                .andThen(new MoveIntakeWristToPositionCommand(wrist, MoPrefs.intakeAgitateHighPos::get))
                .repeatedly();
    }

    private WristCommands() {
        throw new UnsupportedOperationException("IntakeCommands is a static class");
    }
}
