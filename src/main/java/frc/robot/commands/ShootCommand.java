package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.ControllerInput;
import frc.robot.subsystem.IndexerSubsystem;
import frc.robot.subsystem.KickerSubsystem;

public class ShootCommand extends Command {

    private KickerSubsystem kicker;
    private IndexerSubsystem indexer;

    private ControllerInput input;

    public ShootCommand(KickerSubsystem kicker, IndexerSubsystem indexer, ControllerInput input) {

        this.kicker = kicker;
        this.indexer = indexer;

        this.input = input;

        addRequirements(this.kicker, this.indexer);
    }

    public void doShoot(boolean run) {
        if (run) {
            kicker.run();
            indexer.run();
        } else {
            kicker.stop();
            indexer.stop();
        }
    }

    @Override
    public void execute() {
        doShoot(input.getShootRequest());
    }
}
