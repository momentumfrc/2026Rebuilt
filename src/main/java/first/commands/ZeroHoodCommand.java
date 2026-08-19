package frc.robot.commands;

import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;
import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import frc.robot.MoPrefs;
import frc.robot.subsystem.HoodSubsystem;

public class ZeroHoodCommand extends Command {
    private final HoodSubsystem hood;
    private Timer timer = new Timer();

    public ZeroHoodCommand(HoodSubsystem hood) {
        this.hood = hood;

        addRequirements(hood);
    }

    @Override
    public void initialize() {
        timer.restart();

        hood.disableLimitsForZeroing();
    }

    @Override
    public void execute() {
        hood.setVoltage((Voltage) MoPrefs.hoodZeroPower.get().unaryMinus());

        if (hood.getCurrent().gte((Current) MoPrefs.hoodZeroCurrentThreshold.get())) {
            if (timer.hasElapsed(MoPrefs.hoodZeroTime.get())) {
                hood.zeroEncoder();
            }
        } else {
            timer.restart();
        }
    }

    @Override
    public boolean isFinished() {
        return hood.hasZero();
    }

    @Override
    public void end(boolean interrupted) {
        hood.enableLimits();
    }
}
