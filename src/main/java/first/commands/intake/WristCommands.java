package frc.robot.commands.intake;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
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
                        (Voltage) MoPrefs.intakeWristRevHoldVoltage.get().unaryMinus()));
            case OUT -> wrist.run(() -> wrist.moveVoltage((Voltage) MoPrefs.intakeWristFwdHoldVoltage.get()));
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
                                moveToPositionCommand(wrist, MoPrefs.intakeWristRetractPosition::get)
                                        .andThen(holdIntakeWristCommand(wrist, Direction.IN)),
                                MoPrefs.intakeWristAgitatePeriod::get),
                        Utils.withTimeoutPref(
                                moveToPositionCommand(wrist, MoPrefs.intakeWristDeployPosition::get)
                                        .andThen(holdIntakeWristCommand(wrist, Direction.OUT)),
                                MoPrefs.intakeWristAgitatePeriod::get))
                .withName("AgitateIntakeWristCommand");
    }

    public static Command idleWristCommand(IntakeWristSubsystem wrist) {
        return wrist.run(wrist::stopWristMotor).withName("IdleIntakeWristCommand");
    }

    private WristCommands() {
        throw new UnsupportedOperationException("IntakeCommands is a static class");
    }
}
