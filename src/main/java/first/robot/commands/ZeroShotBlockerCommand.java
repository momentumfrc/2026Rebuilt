package first.robot.commands;

import first.robot.MoPrefs;
import first.robot.subsystem.ShotBlockerSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.system.Timer;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Voltage;

public class ZeroShotBlockerCommand extends Command {
    private final ShotBlockerSubsystem blocker;

    private final Timer spinupTimer = new Timer();
    private final Timer tripTimer = new Timer();

    private enum State {
        SPINUP,
        ZEROING;
    }

    private State state = State.SPINUP;

    public ZeroShotBlockerCommand(ShotBlockerSubsystem blocker) {
        this.blocker = blocker;
        addRequirements(blocker);
    }

    @Override
    public void initialize() {
        spinupTimer.restart();
        tripTimer.stop();
        tripTimer.reset();

        blocker.setEnableWinchReverseSoftLimit(false);
        blocker.invalidateZero();

        state = State.SPINUP;
    }

    @Override
    public void execute() {
        if (blocker.hasZero()) {
            // Should never reach this condition, because if the blocker has a zero the command ends
            blocker.stop();
            return;
        }

        // On the first loop or two, the motor has only just started moving. So we should ignore that the velocity
        // is below the threshold - the motor isn't stalled, it just hasn't had time to get up to speed yet.
        if (state == State.SPINUP) {
            if (spinupTimer.hasElapsed(MoPrefs.shotBlockerSpinupTime.get())) {
                state = State.ZEROING;
                tripTimer.restart();
            }
        }

        if (state == State.ZEROING) {
            if (blocker.getWinchMotorVelocity().abs(Units.RPM)
                    < MoPrefs.shotBlockerStallThreshold.get().in(Units.RPM)) {
                // We're outside of the SPINUP period, and the motor's velocity has dropped below the threshold.
                // The motor is stalled.
                if (tripTimer.hasElapsed(MoPrefs.shotBlockerZeroTimeCutoff.get())) {
                    blocker.setZero(MoPrefs.shotBlockerZeroPosition.get());
                }
            } else {
                tripTimer.restart();
            }
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
