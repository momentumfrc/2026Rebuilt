package frc.robot.command.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.MoPrefs;
import frc.robot.subsystem.IntakeWristSubsystem;

public class WristCommands {
    public enum Direction {
        IN,
        OUT;
    }

    public static Command holdIntakeWristCommand(IntakeWristSubsystem wrist, Direction direction) {
        return switch (direction) {
            case IN -> Commands.run(wrist::holdWristIn, wrist);
            case OUT -> Commands.run(wrist::holdWristOut, wrist);
        };
    }

    public static Command moveIntakeWristCommand(IntakeWristSubsystem wrist, Direction direction) {
        var command =
                switch (direction) {
                    case IN -> Commands.run(wrist::wristIn, wrist);
                    case OUT -> Commands.run(wrist::wristOut, wrist);
                };
        return command.withTimeout(MoPrefs.intakeCurrentWristTime.get().in(Units.Seconds));
    }

    public static Command deployIntakeWristCommand(IntakeWristSubsystem wrist) {
        return Commands.deadline(
                moveIntakeWristCommand(wrist, Direction.OUT).andThen(holdIntakeWristCommand(wrist, Direction.OUT)));
    }

    public static Command retractIntakeWristCommand(IntakeWristSubsystem wrist) {
        return Commands.deadline(
                moveIntakeWristCommand(wrist, Direction.OUT).andThen(holdIntakeWristCommand(wrist, Direction.IN)));
    }

    public static Command intakeWristDefaultCommand(IntakeWristSubsystem wrist) {
        return moveIntakeWristCommand(wrist, Direction.IN)
                .andThen(WristCommands.holdIntakeWristCommand(wrist, Direction.IN));
    }

    private WristCommands() {
        throw new UnsupportedOperationException("IntakeCommands is a static class");
    }
}
