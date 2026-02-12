// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootCommand;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.HoodSubsystem;
import frc.robot.subsystem.IndexerSubsystem;
import frc.robot.subsystem.KickerSubsystem;
import frc.robot.subsystem.PositioningSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();
    private final PositioningSubsystem visionSubsystem =
            new PositioningSubsystem(driveSubsystem.getSwerveDrive(), turretSubsystem::getTurretLimelightPose);
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();

    private Trigger resetFieldOrientedFwd;

    private final MoInput input = new ControllerInput();

    private final SwerveInputStream driveAngularVelocity;

    private final Command driveFieldOrientedAngularVelocity;
    private final ShootCommand shootCommand;

    public RobotContainer() {
        driveAngularVelocity = SwerveInputStream.of(
                        driveSubsystem.getSwerveDrive(),
                        () -> input.getDriveMoveXRequest(),
                        () -> input.getDriveMoveYRequest())
                .withControllerRotationAxis(input::getDriveTurnRequest)
                .allianceRelativeControl(true);

        MoPrefs.inputDeadband.subscribe(deadband -> driveAngularVelocity.deadband(deadband), true);

        driveFieldOrientedAngularVelocity = driveSubsystem.driveFieldOriented(driveAngularVelocity);

        shootCommand = new ShootCommand(driveSubsystem, kicker, indexer, hood, input);

        configureBindings();
        setDefaultCommands();
    }

    public void setDefaultCommands() {
        driveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
        hood.setDefaultCommand(shootCommand);
        indexer.setDefaultCommand(shootCommand);
        kicker.setDefaultCommand(shootCommand);
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
