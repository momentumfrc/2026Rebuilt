// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();

    private Trigger resetFieldOrientedFwd;

    private final MoInput input = new ControllerInput();

    private final SwerveInputStream driveAngularVelocity;

    private final Command driveFieldOrientedAngularVelocity;

    public RobotContainer() {
        driveAngularVelocity = SwerveInputStream.of(
                        driveSubsystem.getSwerveDrive(),
                        () -> input.getDriveMoveXRequest(),
                        () -> input.getDriveMoveYRequest())
                .withControllerRotationAxis(input::getDriveTurnRequest)
                .allianceRelativeControl(true);

        MoPrefs.inputDeadband.subscribe(deadband -> driveAngularVelocity.deadband(deadband), true);

        driveFieldOrientedAngularVelocity = driveSubsystem.driveFieldOriented(driveAngularVelocity);

        configureBindings();
        setDefaultCommands();
    }

    public void setDefaultCommands() {
        driveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    private void configureBindings() {
        // Drive Triggers
        resetFieldOrientedFwd = new Trigger(() -> input.getReZeroGyro());

        // Drive Trigger Bindings
        resetFieldOrientedFwd.onTrue(driveSubsystem.resetFieldOrientedFwd());
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
