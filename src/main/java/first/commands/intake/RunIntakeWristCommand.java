package frc.robot.commands.intake;

import frc.robot.MoPrefs;
import frc.robot.commands.intake.WristCommands.Direction;
import frc.robot.input.MoInput;
import frc.robot.molib.NTHelpers;
import frc.robot.subsystem.IntakeWristSubsystem;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.networktables.StringPublisher;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Voltage;

public class RunIntakeWristCommand extends Command {
    private final IntakeWristSubsystem wrist;
    private final Supplier<MoInput> inputSupplier;
    private final Timer moveTimer = new Timer();

    private enum WristState {
        AT_POSITION,
        MOVING;
    }

    private WristCommands.Direction currDirection = Direction.IN;
    private WristState currState = WristState.MOVING;

    private final StringPublisher directionPublisher;
    private final StringPublisher statePublisher;

    public RunIntakeWristCommand(IntakeWristSubsystem wrist, Supplier<MoInput> inputSupplier) {
        this.wrist = wrist;
        this.inputSupplier = inputSupplier;

        final var table = NTHelpers.getTable("Intake Wrist");
        directionPublisher = table.getStringTopic("Wrist Direction").publish();
        statePublisher = table.getStringTopic("Wrist State").publish();

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        currState = WristState.MOVING;
        moveTimer.restart();
    }

    @Override
    public void execute() {
        var input = inputSupplier.get();
        WristCommands.Direction requestedDirection = currDirection;
        if (input.getRetractIntake()) {
            requestedDirection = Direction.IN;
        } else if (input.getExtendIntake()) {
            requestedDirection = Direction.OUT;
        }

        if (requestedDirection != currDirection) {
            currState = WristState.MOVING;
            currDirection = requestedDirection;
            moveTimer.restart();
        }

        if (currState == WristState.MOVING) {
            var targetPosition =
                    switch (currDirection) {
                        case OUT -> MoPrefs.intakeWristDeployPosition.get();
                        case IN -> MoPrefs.intakeWristRetractPosition.get();
                    };
            if (wrist.atPosition(targetPosition) || moveTimer.hasElapsed(MoPrefs.intakeWristMoveTimeout.get())) {
                currState = WristState.AT_POSITION;
            } else {
                wrist.movePosition(targetPosition);
            }
        }

        if (currState == WristState.AT_POSITION) {
            if (currDirection == Direction.OUT) {
                if (MoPrefs.intakeWristFwdHoldApplyFF.get()) {
                    wrist.moveVoltage((Voltage) MoPrefs.intakeWristFwdHoldVoltage.get());
                } else {
                    wrist.moveVoltageNoFF((Voltage) MoPrefs.intakeWristFwdHoldVoltage.get());
                }
            } else {
                wrist.moveVoltage(
                        (Voltage) MoPrefs.intakeWristRevHoldVoltage.get().unaryMinus());
            }
        }

        directionPublisher.accept(currDirection.toString());
        statePublisher.accept(currState.toString());
    }
}
