package first.robot.commands;

import first.robot.MoPrefs;
import first.robot.input.MoInput;
import first.robot.subsystem.ShotBlockerSubsystem;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.units.measure.Voltage;

public class ShotBlockerCommand extends Command {

    private ShotBlockerSubsystem shotBlocker;
    private Supplier<MoInput> inputSupplier;

    public ShotBlockerCommand(ShotBlockerSubsystem climb, Supplier<MoInput> inputSupplier) {
        this.shotBlocker = climb;
        this.inputSupplier = inputSupplier;

        addRequirements(climb);
    }

    @Override
    public void execute() {
        MoInput input = inputSupplier.get();

        double shotBlockerRequest = input.getShotBlockerRequest();

        shotBlocker.runWinch((Voltage) MoPrefs.shotBlockerMaxVoltage
                .get()
                .times(shotBlockerRequest)); // there must be a better way to do this but it's fine
    }

    @Override
    public void end(boolean interrupted) {
        this.shotBlocker.stop();
    }
}
