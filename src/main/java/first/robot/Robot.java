// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import first.robot.molib.NTHelpers;
import first.robot.molib.motune.MoTuner;
import first.robot.molib.prefs.MoPrefsImpl;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;
import org.wpilib.system.DataLogManager;

public class Robot extends TimedRobot {
    private Command autonomousCommand;

    private final RobotContainer robotContainer;

    public Robot() {
        robotContainer = new RobotContainer();

        DataLogManager.start();

        MoPrefsImpl.cleanUpPrefs();

        PathfindingCommand.warmupCommand();
    }

    @Override
    public void robotPeriodic() {
        // Update latest robot position before anything else runs
        robotContainer.robotPositioning.update();
        robotContainer.checkRumbles();

        CommandScheduler.getInstance().run();

        MoTuner.pollAllStateValues();
        NTHelpers.updateSendables();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autonomousCommand = robotContainer.getAutonomousCommand();

        if (autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void utilityInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void utilityPeriodic() {}

    @Override
    public void utilityExit() {}
}
