package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
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

    // TODO: put actual coords
    public static final Translation2d TARGET_POSITION = new Translation2d(2, 2);

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
        if (run) {
            hood.setCalculatedPosition(drive.getRobotPosition(), TARGET_POSITION);
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
