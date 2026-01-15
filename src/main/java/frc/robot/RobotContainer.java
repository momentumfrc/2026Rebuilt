// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystem.DriveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
    private final XboxController driveController = new XboxController(Constants.DRIVE_CONTORLLER_PORT.hidport());

    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    private Trigger resetFieldOrientedFwd;

    SwerveInputStream driveAngularVelocity = SwerveInputStream.of(
                    driveSubsystem.getSwerveDrive(),
                    () -> driveController.getLeftY() * 1,
                    () -> driveController.getLeftX() * 1)
            .withControllerRotationAxis(driveController::getRightX)
            .deadband(Constants.DEADBAND)
            .allianceRelativeControl(true);

    Command driveFieldOrientedAngularVelocity = driveSubsystem.driveFieldOriented(driveAngularVelocity);

    public RobotContainer() {
        configureBindings();
        setDefaultCommands();
    }

    public void setDefaultCommands() {
        driveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    private void configureBindings() {
        // Drive Triggers
        resetFieldOrientedFwd = new Trigger(() -> driveController.getBackButton());

        // Drive Trigger Bindings
        resetFieldOrientedFwd.onTrue(driveSubsystem.resetFieldOrientedFwd());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
