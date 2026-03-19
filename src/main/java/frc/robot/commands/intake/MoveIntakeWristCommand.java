package frc.robot.commands.intake;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MoPrefs;
import frc.robot.subsystem.IntakeWristSubsystem;

public class MoveIntakeWristCommand extends Command {
    private final IntakeWristSubsystem wrist;
    private final WristCommands.Direction direction;

    private final Timer currentSenseTimer = new Timer();
    private final Timer startupTimer = new Timer();

    public MoveIntakeWristCommand(IntakeWristSubsystem wrist, WristCommands.Direction direction) {
        this.wrist = wrist;
        this.direction = direction;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        currentSenseTimer.restart();
        startupTimer.restart();
    }

    private Voltage getVoltage() {
        return (Voltage)
                switch (direction) {
                    case IN -> MoPrefs.intakeWristVoltage.get().unaryMinus();
                    case OUT -> MoPrefs.intakeWristVoltage.get();
                };
    }

    @Override
    public void execute() {
        wrist.moveVoltage(getVoltage());
    }

    @Override
    public boolean isFinished() {
        if(startupTimer.hasElapsed(MoPrefs.intakeWristMoveMinTime.get()) == false) {
            currentSenseTimer.restart();
            return false;
        }
        if (wrist.getIntakeWristCurrent().gte(MoPrefs.intakeWristCurrentThresh.get())) {
            return currentSenseTimer.hasElapsed(MoPrefs.intakeWristCurrentTime.get());
        }
        currentSenseTimer.restart();
        return false;
    }
}
