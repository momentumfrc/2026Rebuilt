package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MoPrefs;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.HoodSubsystem;
import frc.robot.util.OdometryTargetingHelper;

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
        targetDistance.mut_replace(
                OdometryTargetingHelper.getTranslationToTarget(
                                drive.getRobotPosition(),
                                DriverStation.getAlliance().orElse(Alliance.Red))
                        .getNorm(),
                Units.Meters);
    }

    // will there be one?
    public boolean isInDeadzone() {
        return true;
    }
}
