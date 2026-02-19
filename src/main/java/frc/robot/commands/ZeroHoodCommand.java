package frc.robot.commands;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
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
        hood.setVoltage((Voltage) MoPrefs.hoodZeroPower.get());

        if (hood.getCurrent().gte((Current) MoPrefs.hoodCurrentLimit.get())) {
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
