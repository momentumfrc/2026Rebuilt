package frc.robot.commands.intake;

import org.wpilib.units.measure.Voltage;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.robot.MoPrefs;
import frc.robot.subsystem.IntakeWristSubsystem;

public class ZeroIntakeWristCommand extends Command {
    private final IntakeWristSubsystem wrist;

    private final Timer currentSenseTimer = new Timer();

    public ZeroIntakeWristCommand(IntakeWristSubsystem wrist) {
        this.wrist = wrist;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        currentSenseTimer.restart();
    }

    @Override
    public void execute() {
        wrist.moveVoltage((Voltage) MoPrefs.intakeWristZeroVoltage.get().unaryMinus());
    }

    @Override
    public boolean isFinished() {
        if (wrist.getIntakeWristCurrent().gte(MoPrefs.intakeWristZeroThresh.get())) {
            return currentSenseTimer.hasElapsed(MoPrefs.intakeWristZeroTime.get());
        }
        currentSenseTimer.restart();
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted == false) {
            wrist.zeroEncoder();
        }
    }
}
