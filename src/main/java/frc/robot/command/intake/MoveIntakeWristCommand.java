package frc.robot.command.intake;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MoPrefs;
import frc.robot.command.intake.WristCommands.Direction;
import frc.robot.subsystem.IntakeWristSubsystem;

public class MoveIntakeWristCommand extends Command {
    private final IntakeWristSubsystem intakeWristSubsystem;
    private final Direction direction;

    private final Timer wristCurrentTimer = new Timer();

    public MoveIntakeWristCommand(IntakeWristSubsystem intake, Direction direction) {
        this.intakeWristSubsystem = intake;
        this.direction = direction;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        wristCurrentTimer.restart();
    }

    @Override
    public void execute() {
        if (direction == Direction.IN) {
            intakeWristSubsystem.wristIn();
        } else {
            intakeWristSubsystem.wristOut();
        }
    }

    @Override
    public boolean isFinished() {
        return wristCurrentTimer.hasElapsed(MoPrefs.intakeCurrentWristTime.get().in(Units.Seconds));
    }
}
