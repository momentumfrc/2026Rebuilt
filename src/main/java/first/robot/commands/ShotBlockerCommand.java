package first.robot.commands;

import first.robot.MoPrefs;
import first.robot.input.MoInput;
import first.robot.subsystem.ShotBlockerSubsystem;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.units.Measure;
import org.wpilib.units.Units;
import org.wpilib.units.VoltageUnit;
import org.wpilib.units.measure.Voltage;

public class ShotBlockerCommand extends Command {
    private static final double TARGET_TOLERANCE = 0.2;

    private ShotBlockerSubsystem shotBlocker;
    private Supplier<MoInput> inputSupplier;

    private enum Position {
        UP,
        DOWN;
    }

    private enum State {
        STATIONARY,
        MOVING;
    }

    private Position position = null;
    private State state = State.MOVING;

    public ShotBlockerCommand(ShotBlockerSubsystem climb, Supplier<MoInput> inputSupplier) {
        this.shotBlocker = climb;
        this.inputSupplier = inputSupplier;

        addRequirements(climb);
    }

    @Override
    public void initialize() {
        position = null;
        state = State.MOVING;
    }

    @Override
    public void execute() {
        var requestedPosition = inputSupplier.get().getRetractShotBlocker() ? Position.DOWN : Position.UP;
        if (requestedPosition != position) {
            state = State.MOVING;
            position = requestedPosition;
        }

        double targetHeight =
                switch (position) {
                    case DOWN -> 0;
                    case UP -> MoPrefs.shotBlockerMaxHeight.get();
                };

        if (state == State.MOVING && Math.abs(shotBlocker.getCurrentHeight() - targetHeight) < TARGET_TOLERANCE) {
            state = State.STATIONARY;
        }

        Measure<VoltageUnit> outputPower =
                switch (state) {
                    case MOVING ->
                        switch (position) {
                            case DOWN -> MoPrefs.shotBlockerReversePower.get();
                            case UP -> MoPrefs.shotBlockerForwardPower.get();
                        };
                    case STATIONARY ->
                        switch (position) {
                            case DOWN -> MoPrefs.shotBlockerHoldPower.get();
                            case UP -> Units.Volts.zero();
                        };
                };

        shotBlocker.runWinch((Voltage) outputPower);
    }

    @Override
    public void end(boolean interrupted) {
        this.shotBlocker.stop();
    }
}
