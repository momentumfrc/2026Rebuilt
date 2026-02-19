package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.HoodSubsystem;
import frc.robot.subsystem.IndexerSubsystem;
import frc.robot.subsystem.KickerSubsystem;

public class ShootCommand extends Command {

    private KickerSubsystem kicker;
    private IndexerSubsystem indexer;
    private HoodSubsystem hood;
    private DriveSubsystem drive;

    private MoInput input;

    private MutDistance targetDistance = Units.Meters.mutable(0);

    public ShootCommand(
            DriveSubsystem drive, KickerSubsystem kicker, IndexerSubsystem indexer, HoodSubsystem hood, MoInput input) {

        this.kicker = kicker;
        this.indexer = indexer;
        this.hood = hood;
        this.drive = drive;

        this.input = input;

        addRequirements(this.kicker, this.indexer, this.hood, this.drive);
    }

    public void doShoot(boolean run) {
        if (run && hood.isInPosition()) {
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
        //TODO: mut_replace
    }
}
