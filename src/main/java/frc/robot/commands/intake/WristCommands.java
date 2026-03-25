package frc.robot.commands.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.MoPrefs;
import frc.robot.molib.Utils;
import frc.robot.subsystem.IntakeWristSubsystem;
import java.util.function.Supplier;

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

    public static Command moveToPositionCommand(IntakeWristSubsystem wrist, Supplier<Angle> positionSupplier) {
        return Utils.withTimeoutPref(
                wrist.run(() -> wrist.movePosition(positionSupplier.get()))
                        .until(() -> wrist.atPosition(positionSupplier.get())),
                MoPrefs.intakeWristMoveTimeout::get);
    }

    public static Command agitateWristCommand(IntakeWristSubsystem wrist) {
        return Commands.repeatingSequence(
                Utils.withTimeoutPref(
                        moveToPositionCommand(wrist, MoPrefs.intakeWristDeployPosition::get)
                                .andThen(holdIntakeWristCommand(wrist, Direction.OUT)),
                        MoPrefs.intakeWristAgitatePeriod::get),
                Utils.withTimeoutPref(
                        moveToPositionCommand(wrist, MoPrefs.intakeWristRetractPosition::get)
                                .andThen(holdIntakeWristCommand(wrist, Direction.IN)),
                        MoPrefs.intakeWristAgitatePeriod::get));
    }

    private WristCommands() {
        throw new UnsupportedOperationException("IntakeCommands is a static class");
    }
}
