// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;
import org.wpilib.system.DataLogManager;
import org.wpilib.framework.TimedRobot;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import frc.robot.molib.NTHelpers;
import frc.robot.molib.motune.MoTuner;
import frc.robot.molib.prefs.MoPrefsImpl;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    public Robot() {
        m_robotContainer = new RobotContainer();

        DataLogManager.start();

        MoPrefsImpl.cleanUpPrefs();
    }

    @Override
    public void robotInit() {
        PathfindingCommand.warmupCommand();
    }

    @Override
    public void robotPeriodic() {
        // Update latest robot position before anything else runs.
        m_robotContainer.robotPositioning.update();
        m_robotContainer.checkRumbles();

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
        m_robotContainer.setAutoDefaultCommnds();

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        m_robotContainer.setDefaultCommands();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
