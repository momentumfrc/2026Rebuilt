package first.robot.commands;

import first.robot.MoPrefs;
import first.robot.subsystem.ShotBlockerSubsystem;
import first.robot.util.AutomaticZeroArm;
import org.wpilib.command2.Command;
import org.wpilib.system.Timer;
import org.wpilib.units.Units;

public class ZeroShotBlockerCommand extends Command {
    private final ShotBlockerSubsystem climb;

    private Timer currentTimer = new Timer();

    public ZeroShotBlockerCommand(ShotBlockerSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        currentTimer.stop();
        currentTimer.reset();

        climb.arm.enableWinchSoftLimitReverse(false);
        climb.arm.invalidateZero();
    }

    private static void zeroSide(double power, AutomaticZeroArm climber, Timer currentTimer) {
        if (climber.hasZero()) {
            climber.runWinch(0);

            return;
        }

        if (climber.winch.getOutputCurrent().get()
                >= MoPrefs.shotBlockerZeroCurrentCutoff.get().in(Units.Amps)) {
            if (currentTimer.hasElapsed(MoPrefs.shotBlockerZeroTimeCutoff.get().in(Units.Seconds))) {
                climber.setZero(MoPrefs.shotBlockerZeroPosition.get());
            }
        } else {
            currentTimer.restart();
        }

        climber.runWinch(-Math.abs(MoPrefs.shotBlockerZeroPower.get())); // originally intakeZeroPwr, I have no idea why
    }

    @Override
    public void execute() {
        zeroSide(MoPrefs.shotBlockerZeroPower.get(), climb.arm, currentTimer);
    }

    @Override
    public boolean isFinished() {
        return climb.arm.hasZero();
    }

    @Override
    public void end(boolean interrupted) {
        climb.arm.enableWinchSoftLimitReverse(true);
    }
}
