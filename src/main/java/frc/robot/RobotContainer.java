// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.intake.RollerCommands;
import frc.robot.command.intake.WristCommands;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.IntakeRollerSubsystem;
import frc.robot.subsystem.IntakeWristSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import swervelib.SwerveInputStream;

public class RobotContainer {
    // Drive
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final TurretSubsystem turretSubsystem = new TurretSubsystem();

    public final RobotPositioning robotPositioning = new RobotPositioning(
            driveSubsystem.getSwerveDrive(),
            turretSubsystem::getTimestampedTurretYaw,
            turretSubsystem::getTurretYawRate);

    private Trigger resetFieldOrientedFwd;

    private final SwerveInputStream driveAngularVelocity;

    private final Command driveFieldOrientedAngularVelocity;

    // Intake
    private final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
    private final IntakeWristSubsystem intakeWristSubsystem = new IntakeWristSubsystem();

    private final Command runRollerCommand = RollerCommands.runIntakeRollerCommand(intakeRollerSubsystem);

    private final Command extendIntakeWristCommand = WristCommands.deployIntakeWristCommand(intakeWristSubsystem);
    private final Command retractIntakeWristCommend = WristCommands.retractIntakeWristCommand(intakeWristSubsystem);

    private final Command intakeRollerDefaultCommand = RollerCommands.idleIntakeRollerCommand(intakeRollerSubsystem);
    private final Command intakeWristDefaultCommand = WristCommands.intakeWristDefaultCommand(intakeWristSubsystem);

    private Trigger runIntakeTrigger;
    private Trigger extendIntakeTrigger;
    private Trigger retractIntakeTrigger;

    private final MoInput input = new ControllerInput();

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
        intakeRollerSubsystem.setDefaultCommand(intakeRollerDefaultCommand);
        intakeWristSubsystem.setDefaultCommand(intakeWristDefaultCommand);
    }

    private void configureBindings() {
        // Drive Triggers
        resetFieldOrientedFwd = new Trigger(() -> input.getReZeroGyro());

        // Intake Triggers
        runIntakeTrigger = new Trigger(() -> input.getRunIntake());
        extendIntakeTrigger = new Trigger(() -> input.getExtendIntake());
        retractIntakeTrigger = new Trigger(() -> input.getRetractIntake());

        // Drive Trigger Bindings
        resetFieldOrientedFwd.onTrue(driveSubsystem.resetFieldOrientedFwd());

        // Intake Trigger Bindings
        runIntakeTrigger.whileTrue(runRollerCommand);
        extendIntakeTrigger.onTrue(extendIntakeWristCommand);
        retractIntakeTrigger.onTrue(retractIntakeWristCommend);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
