// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ZeroHoodCommand;
import frc.robot.commands.intake.RollerCommands;
import frc.robot.commands.intake.WristCommands;
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
import frc.robot.util.SysIdUtil;
import java.util.List;
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

    private final MoInput controllerInput = new ControllerInput();
    private final SysIdUtil sysId = new SysIdUtil(List.of(
            indexer.getSysIdMechanism(),
            kicker.getSysIdMechanism(),
            turret.getSysIdMechanism(),
            shooter.getSysIdMechanism(),
            hood.getSysIdMechanism()));

    private final SendableChooser<SwerveInputStream> driveModeChooser = new SendableChooser<>();

    // **** COMMANDS ****
    private final Command driveCommand = driveSubsystem.driveFieldOriented(() -> driveModeChooser.getSelected());

    private final ShootCommand shootCommand = new ShootCommand(turretTargetingHelper, kicker, turret, shooter, hood);

    private final Command idleIndexerCommand = indexer.run(indexer::stop);
    private final Command idleKickerCommand = kicker.run(kicker::stop);
    private final Command idleShooterCommand = shooter.run(shooter::stop);
    private final Command idleHoodCommand = hood.run(() -> hood.setPosition(MoPrefs.hoodDeadzonePosition.get()));
    private final Command zeroHoodCommand = new ZeroHoodCommand(hood);

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

    private Trigger zeroHoodTrigger;

    private Trigger runSysIdTrigger;

    public RobotContainer() {
        setupDriveModes();
        configureBindings();
        setDefaultCommands();
        addSubsystemsToDashboard();
    }

    private void setupDriveModes() {
        var swerveInputStreamBase = SwerveInputStream.of(
                        driveSubsystem.getSwerveDrive(), () -> getInput().getDriveMoveXRequest(), () -> getInput()
                                .getDriveMoveYRequest())
                .allianceRelativeControl(true)
                .cubeTranslationControllerAxis(() -> MoPrefs.inputTranslationCubed.get())
                .cubeRotationControllerAxis(() -> MoPrefs.inputRotationCubed.get());

        var driveAngularVelocity = swerveInputStreamBase.copy().withControllerRotationAxis(() -> getInput()
                .getDriveTurnRequest());

        var driveHeading = swerveInputStreamBase
                .copy()
                .withControllerHeadingAxis(() -> getInput().getDriveHeadingXRequest(), () -> getInput()
                        .getDriveHeadingYRequest());

        MoPrefs.inputDeadband.subscribe(
                deadband -> {
                    driveAngularVelocity.deadband(deadband);
                    driveHeading.deadband(deadband);
                },
                true);
        MoPrefs.inputTranslationScale.subscribe(
                scale -> {
                    driveAngularVelocity.scaleTranslation(scale);
                    driveHeading.scaleTranslation(scale);
                },
                true);
        MoPrefs.inputRotationScale.subscribe(scale -> {
            driveAngularVelocity.scaleRotation(scale);
            driveHeading.scaleRotation(scale);
        });

        driveModeChooser.setDefaultOption("Angular Velocity", driveAngularVelocity);
        driveModeChooser.addOption("Heading", driveHeading);
    }

    private void setDefaultCommands() {
        driveSubsystem.setDefaultCommand(driveCommand);
        hood.setDefaultCommand(idleHoodCommand);
        indexer.setDefaultCommand(idleIndexerCommand);
        kicker.setDefaultCommand(idleKickerCommand);
        shooter.setDefaultCommand(idleShooterCommand);
        turret.setDefaultCommand(passiveTargetingCommand);
        intakeRollerSubsystem.setDefaultCommand(intakeRollerDefaultCommand);
        intakeWristSubsystem.setDefaultCommand(intakeWristDefaultCommand);
    }

    private void addSubsystemsToDashboard() {
        SmartDashboard.putData(driveSubsystem);
        SmartDashboard.putData(turret);
        SmartDashboard.putData(indexer);
        SmartDashboard.putData(kicker);
        SmartDashboard.putData(shooter);
        SmartDashboard.putData(hood);
        SmartDashboard.putData(intakeRollerSubsystem);
        SmartDashboard.putData(intakeWristSubsystem);
    }

    private void configureBindings() {
        // Drive Triggers
        resetFieldOrientedFwd = new Trigger(() -> getInput().getReZeroGyro());

        // Intake Triggers
        runIntakeTrigger = new Trigger(() -> getInput().getRunIntake());
        extendIntakeTrigger = new Trigger(() -> getInput().getExtendIntake());
        retractIntakeTrigger = new Trigger(() -> getInput().getRetractIntake());

        shootTrigger = new Trigger(() -> getInput().getShootRequest());

        zeroHoodTrigger = new Trigger(() -> hood.hasZero() == false);

        runSysIdTrigger = new Trigger(() -> getInput().getRunSysId());

        // Drive Trigger Bindings
        resetFieldOrientedFwd.onTrue(driveSubsystem.resetFieldOrientedFwd());

        // Intake Trigger Bindings
        runIntakeTrigger.whileTrue(runRollerCommand);
        extendIntakeTrigger.onTrue(extendIntakeWristCommand);
        retractIntakeTrigger.onTrue(retractIntakeWristCommend);

        shootTrigger.whileTrue(shootCommand);

        zeroHoodTrigger.and(RobotModeTriggers.disabled().negate()).onTrue(zeroHoodCommand);

        runSysIdTrigger.whileTrue(sysId.getSysIdCommand());
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
