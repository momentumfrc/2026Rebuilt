package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.HoodSubsystem;

public class HoodAdjustCommand extends Command {

    private HoodSubsystem hood;
    private DriveSubsystem drive;

    // TODO: put actual coords
    public static final Translation2d TARGET_POSITION = new Translation2d(2, 2);

    public HoodAdjustCommand(HoodSubsystem hood, DriveSubsystem drive) {
        this.hood = hood;
        this.drive = drive;

        addRequirements(this.hood);
    }

    @Override
    public void execute() {
        if (!isInDeadzone()) {
            hood.setCalculatedPosition(drive.getRobotPosition(), TARGET_POSITION);
        }
    }

    // will there be one?
    public boolean isInDeadzone() {
        return true;
    }
}
