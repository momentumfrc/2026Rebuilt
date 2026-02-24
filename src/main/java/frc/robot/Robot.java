// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.molib.motune.MoTuner;
import frc.robot.shootutils.HoodSerializedInformationHolder;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    private final DoubleLogEntry calculatedHoodAngle;
    private final DoubleLogEntry calculatedFlywheelSpeed;

    public Robot() {
        m_robotContainer = new RobotContainer();

        DataLogManager.start();
        DataLog log = DataLogManager.getLog();
        calculatedHoodAngle = new DoubleLogEntry(log, "Calculated Hood Angle");
        calculatedFlywheelSpeed = new DoubleLogEntry(log, "Calculated Flywheel Speed");
    }

    @Override
    public void robotPeriodic() {
        // Update latest robot position before anything else runs.
        m_robotContainer.robotPositioning.update();

        calculatedHoodAngle.append(HoodSerializedInformationHolder.getInstance().getHoodAngle(m_robotContainer.getDistanceToTarget()).in(HoodSerializedInformationHolder.HOOD_ANGLE_STORE_UNIT));

        calculatedFlywheelSpeed.append(HoodSerializedInformationHolder.getInstance().getFlywheelSpeed(m_robotContainer.getDistanceToTarget()).in(HoodSerializedInformationHolder.FLYWHEEL_SPEED_STORE_UNIT));

        CommandScheduler.getInstance().run();

        MoTuner.pollAllStateValues();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
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
