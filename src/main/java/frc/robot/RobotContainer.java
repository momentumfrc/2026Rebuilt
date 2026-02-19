// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.command.intake.RollerCommands;
import frc.robot.command.intake.WristCommands;
import frc.robot.commands.ShootCommand;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.shootutils.TurretTargeting;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.HoodSubsystem;
import frc.robot.subsystem.IndexerSubsystem;
import frc.robot.subsystem.IntakeRollerSubsystem;
import frc.robot.subsystem.IntakeWristSubsystem;
import frc.robot.subsystem.KickerSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.OdometryTargetingHelper;
import swervelib.SwerveInputStream;

public class RobotContainer {
    // **** SUBSYSTEMS ****
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final TurretSubsystem turret = new TurretSubsystem();
    private final IndexerSubsystem indexer = new IndexerSubsystem();
    private final KickerSubsystem kicker = new KickerSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final HoodSubsystem hood = new HoodSubsystem();
    private final IntakeRollerSubsystem intakeRollerSubsystem = new IntakeRollerSubsystem();
    private final IntakeWristSubsystem intakeWristSubsystem = new IntakeWristSubsystem();

    // **** UTILITIES ****
    public final RobotPositioning robotPositioning = new RobotPositioning(
            driveSubsystem.getSwerveDrive(),
            () -> getTurretSubsystem().getTimestampedTurretYaw(),
            () -> getTurretSubsystem().getTurretYawRate());

    private final TurretTargeting turretTargetingHelper = new TurretTargeting(robotPositioning);

    private final SwerveInputStream driveAngularVelocity;

    // **** COMMANDS ****
    private final Command driveFieldOrientedAngularVelocity;

    private final ShootCommand shootCommand = new ShootCommand(turretTargetingHelper, kicker, turret, shooter, hood);

    private final Command idleIndexerCommand = indexer.run(indexer::stop);
    private final Command idleKickerCommand = kicker.run(kicker::stop);
    private final Command idleShooterCommand = shooter.run(shooter::stop);
    private final Command idleHoodCommand = hood.run(() -> hood.setPosition(MoPrefs.hoodDeadzonePosition.get()));

    private final Command passiveTargetingCommand = turret.run(() -> {
        var target =
                OdometryTargetingHelper.getTarget(DriverStation.getAlliance().orElse(DriverStation.Alliance.Red));
        var firingSolution = turretTargetingHelper.targetPosition(target.toTranslation2d());
        turret.align(firingSolution);
    });

    private final Command runRollerCommand = RollerCommands.runIntakeRollerCommand(intakeRollerSubsystem);
    private final Command extendIntakeWristCommand = WristCommands.deployIntakeWristCommand(intakeWristSubsystem);
    private final Command retractIntakeWristCommend = WristCommands.retractIntakeWristCommand(intakeWristSubsystem);
    private final Command intakeRollerDefaultCommand = RollerCommands.idleIntakeRollerCommand(intakeRollerSubsystem);
    private final Command intakeWristDefaultCommand = WristCommands.intakeWristDefaultCommand(intakeWristSubsystem);

    // **** TRIGGERS ****
    private Trigger resetFieldOrientedFwd;

    private Trigger runIntakeTrigger;
    private Trigger extendIntakeTrigger;
    private Trigger retractIntakeTrigger;

    private Trigger shootTrigger;

    // **** MISC ****
    private final MoInput controllerInput = new ControllerInput();

    public RobotContainer() {
        driveAngularVelocity = SwerveInputStream.of(
                        driveSubsystem.getSwerveDrive(), () -> getInput().getDriveMoveXRequest(), () -> getInput()
                                .getDriveMoveYRequest())
                .withControllerRotationAxis(() -> getInput().getDriveTurnRequest())
                .allianceRelativeControl(true);

        MoPrefs.inputDeadband.subscribe(deadband -> driveAngularVelocity.deadband(deadband), true);

        driveFieldOrientedAngularVelocity = driveSubsystem.driveFieldOriented(driveAngularVelocity);

        configureBindings();
        setDefaultCommands();
    }

    public void setDefaultCommands() {
        driveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocity);
        hood.setDefaultCommand(idleHoodCommand);
        indexer.setDefaultCommand(idleIndexerCommand);
        kicker.setDefaultCommand(idleKickerCommand);
        shooter.setDefaultCommand(idleShooterCommand);
        turret.setDefaultCommand(passiveTargetingCommand);
        intakeRollerSubsystem.setDefaultCommand(intakeRollerDefaultCommand);
        intakeWristSubsystem.setDefaultCommand(intakeWristDefaultCommand);
    }

    private void configureBindings() {
        // Drive Triggers
        resetFieldOrientedFwd = new Trigger(() -> getInput().getReZeroGyro());

        // Intake Triggers
        runIntakeTrigger = new Trigger(() -> getInput().getRunIntake());
        extendIntakeTrigger = new Trigger(() -> getInput().getExtendIntake());
        retractIntakeTrigger = new Trigger(() -> getInput().getRetractIntake());

        shootTrigger = new Trigger(() -> getInput().getShootRequest());

        // Drive Trigger Bindings
        resetFieldOrientedFwd.onTrue(driveSubsystem.resetFieldOrientedFwd());

        // Intake Trigger Bindings
        runIntakeTrigger.whileTrue(runRollerCommand);
        extendIntakeTrigger.onTrue(extendIntakeWristCommand);
        retractIntakeTrigger.onTrue(retractIntakeWristCommend);

        shootTrigger.whileTrue(shootCommand);
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }

    private MoInput getInput() {
        return controllerInput;
    }

    private TurretSubsystem getTurretSubsystem() {
        return turret;
    }
}
