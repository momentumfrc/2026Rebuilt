package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MoPrefs;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.HoodSubsystem;

public class HoodAdjustCommand extends Command {

    private HoodSubsystem hood;
    private DriveSubsystem drive;

    private MutDistance targetDistance = Units.Meters.mutable(0);

    public HoodAdjustCommand(HoodSubsystem hood, DriveSubsystem drive) {
        this.hood = hood;
        this.drive = drive;

        addRequirements(this.hood);
    }

    @Override
    public void execute() {
        if (!isInDeadzone()) {
            hood.setCalculatedPosition(targetDistance);
        } else {
            hood.setPosition(MoPrefs.hoodDeadzonePosition.get());
        }
        // TODO: mut_replace
    }

    // will there be one?
    public boolean isInDeadzone() {
        return true;
    }
}
