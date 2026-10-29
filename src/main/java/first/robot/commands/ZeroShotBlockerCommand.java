package first.robot.commands;

import first.robot.MoPrefs;
import first.robot.subsystem.ShotBlockerSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Voltage;

public class ZeroShotBlockerCommand extends Command {
    private final ShotBlockerSubsystem blocker;

    private Timer currentTimer = new Timer();

    public ZeroShotBlockerCommand(ShotBlockerSubsystem blocker) {
        this.blocker = blocker;
        addRequirements(blocker);
    }

    @Override
    public void initialize() {
        currentTimer.stop();
        currentTimer.reset();

        blocker.setEnableWinchReverseSoftLimit(false);
        blocker.invalidateZero();
    }

    @Override
    public void execute() {
        if (blocker.hasZero()) {
            blocker.stop();
            return;
        }

        if (blocker.getWinchMotorCurrent().gte(MoPrefs.shotBlockerZeroCurrentCutoff.get())) {
            if (currentTimer.hasElapsed(MoPrefs.shotBlockerZeroTimeCutoff.get())) {
                blocker.setZero(MoPrefs.shotBlockerZeroPosition.get());
            }
        } else {
            currentTimer.restart();
        }

        blocker.runWinch((Voltage) MoPrefs.shotBlockerZeroPower.get());
    }

    @Override
    public boolean isFinished() {
        return blocker.hasZero();
    }

    @Override
    public void end(boolean interrupted) {
        blocker.setEnableWinchReverseSoftLimit(true);
    }
}
