// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.LEDCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ZeroHoodCommand;
import frc.robot.commands.intake.RollerCommands;
import frc.robot.commands.intake.WristCommands;
import frc.robot.commands.intake.ZeroIntakeWristCommand;
import frc.robot.input.ControllerInput;
import frc.robot.input.MoInput;
import frc.robot.molib.NTHelpers;
import frc.robot.molib.Utils;
import frc.robot.shootutils.TurretTargeting;
import frc.robot.subsystem.DriveSubsystem;
import frc.robot.subsystem.HoodSubsystem;
import frc.robot.subsystem.IndexerSubsystem;
import frc.robot.subsystem.IntakeRollerSubsystem;
import frc.robot.subsystem.IntakeWristSubsystem;
import frc.robot.subsystem.KickerSubsystem;
import frc.robot.subsystem.LEDSubsystem;
import frc.robot.subsystem.ShooterSubsystem;
import frc.robot.subsystem.TurretSubsystem;
import frc.robot.util.SysIdUtil;
import java.util.Collections;
import java.util.List;

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
    private final LEDSubsystem leds = new LEDSubsystem();

    // **** UTILITIES ****
    public final RobotPositioning robotPositioning = new RobotPositioning(
            driveSubsystem.getSwerveDrive(), () -> getTurretSubsystem().getTurretYaw(), () -> getTurretSubsystem()
                    .getTurretYawRate());

    private final TurretTargeting turretTargetingHelper = new TurretTargeting(robotPositioning);

    private final ControllerInput controllerInput = new ControllerInput();
    private final SysIdUtil sysId = new SysIdUtil(List.of(
            intakeWristSubsystem.getSysIdMechanism(),
            kicker.getSysIdMechanism(),
            turret.getSysIdMechanism(),
            shooter.getSysIdMechanism(),
            hood.getSysIdMechanism()));

    // **** COMMANDS ****
    private final Command driveCommand = driveSubsystem.getTeleopDriveCommand(this::getInput);

    private final ShootCommand shootCommand =
            new ShootCommand(turretTargetingHelper, indexer, kicker, turret, shooter, hood);

    private final Command shooterTestCommand = shooter.getTestCommand(controllerInput.getOperatorController());

    private final Command idleIndexerCommand = indexer.run(indexer::stop).withName("IdleIndexerCommand");
    private final Command idleKickerCommand = kicker.run(kicker::stop).withName("IdleKickerCommand");
    private final Command idleShooterCommand = shooter.run(shooter::stop).withName("IdleShooterCommand");
    private final Command idleHoodCommand = hood.run(hood::goToRest).withName("IdleHoodCommand");
    private final Command zeroHoodCommand = new ZeroHoodCommand(hood);
    private final Command hoodTestCommand = hood.testCommand(controllerInput.getOperatorController());

    private final Command runIndexerToShootCommand = indexer.run(indexer::run).withName("RunIndexerToShootCommand");
    private final Command runIndexerWithIntakeCommand =
            indexer.run(indexer::run).withName("RunIndexerWithIntakeCommand");
    private final Command runIndexerReverseCommand =
            indexer.run(indexer::runReverse).withName("RunIndexerReverseCommand");

    private final Command passiveTargetingCommand = turret.passiveTargetingCommand(turretTargetingHelper);
    private final Command idleTurretCommand = turret.run(turret::stop);
    private final Command turretTestCommand = turret.testCommand(controllerInput.getOperatorController());

    private final Command runKickerCommand = kicker.run(kicker::run);

    private final Command runRollerCommand = RollerCommands.runIntakeRollerCommand(intakeRollerSubsystem);
    private final Command extendIntakeWristCommand = WristCommands.deployIntakeWristCommand(intakeWristSubsystem);
    private final Command retractIntakeWristCommand = WristCommands.retractIntakeWristCommand(intakeWristSubsystem);
    private final Command intakeRollerDefaultCommand = RollerCommands.idleIntakeRollerCommand(intakeRollerSubsystem);
    private final Command intakeWristDefaultCommand = WristCommands.intakeWristDefaultCommand(intakeWristSubsystem);
    private final Command zeroIntakeWristCommand = new ZeroIntakeWristCommand(intakeWristSubsystem);
    private final Command testIntakeWristCommand = intakeWristSubsystem.testCommand(controllerInput.getOperatorController());

    private final Command ledCommand = new LEDCommand(leds, robotPositioning, turret);

    // **** TRIGGERS ****
    private Trigger resetFieldOrientedFwd;

    private Trigger runIntakeTrigger;
    private Trigger agitateIntakeTrigger;
    private Trigger extendIntakeTrigger;
    private Trigger retractIntakeTrigger;

    private Trigger reverseIndexerTrigger;

    private Trigger clearShooterTrigger;
    private Trigger shootTrigger;

    private Trigger zeroHoodTrigger;
    private Trigger zeroIntakeWristTrigger;

    private Trigger runSysIdTrigger;

    private Trigger lockTrigger;

    public RobotContainer() {
        configureBindings();
        setDefaultCommands();
        addSubsystemsToDashboard();
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
        leds.setDefaultCommand(ledCommand);
    }

    private void addSubsystemsToDashboard() {
        var table = NTHelpers.getTable("subsystems");
        NTHelpers.publishSendable(table, driveSubsystem);
        NTHelpers.publishSendable(table, turret);
        NTHelpers.publishSendable(table, indexer);
        NTHelpers.publishSendable(table, kicker);
        NTHelpers.publishSendable(table, shooter);
        NTHelpers.publishSendable(table, hood);
        NTHelpers.publishSendable(table, intakeRollerSubsystem);
        NTHelpers.publishSendable(table, intakeWristSubsystem);
        NTHelpers.publishSendable(table, leds);
    }

    private void configureBindings() {
        // Drive Triggers
        resetFieldOrientedFwd = new Trigger(() -> getInput().getReZeroGyro());

        // Intake Triggers
        runIntakeTrigger = new Trigger(() -> getInput().getRunIntake());
        agitateIntakeTrigger = new Trigger(() -> getInput().getAgitate());
        extendIntakeTrigger = new Trigger(() -> getInput().getExtendIntake());
        retractIntakeTrigger = new Trigger(() -> getInput().getRetractIntake());

        clearShooterTrigger = new Trigger(() -> getInput().getClearShooter());
        shootTrigger = new Trigger(() -> getInput().getShootRequest());

        zeroHoodTrigger = new Trigger(() -> hood.hasZero() == false);
        zeroIntakeWristTrigger = new Trigger(() -> intakeWristSubsystem.hasZero() == false);

        runSysIdTrigger = new Trigger(() -> getInput().getRunSysId());

        lockTrigger = new Trigger(() -> getInput().getLockRequest());

        reverseIndexerTrigger = new Trigger(() -> getInput().getReverseIndexerRequest());

        // Drive Trigger Bindings
        resetFieldOrientedFwd.onTrue(driveSubsystem.resetFieldOrientedFwd());

        // Intake Trigger Bindings
        runIntakeTrigger.whileTrue(runRollerCommand);
        retractIntakeTrigger
                .or(agitateIntakeTrigger)
                .and(zeroIntakeWristTrigger.negate())
                .onTrue(retractIntakeWristCommand);
        extendIntakeTrigger.and(zeroIntakeWristTrigger.negate()).onTrue(extendIntakeWristCommand);

        shootTrigger.whileTrue(Utils.withTimeoutPref(clearKickerShooterCommand(), MoPrefs.shooterClearTime::get)
                .andThen(shootCommand));
        shootTrigger
                .and(reverseIndexerTrigger.negate())
                .whileTrue(Commands.defer(() -> Commands.waitUntil(shootCommand::readyToShoot), Collections.emptySet())
                        .andThen(runIndexerToShootCommand));

        clearShooterTrigger.and(shootTrigger.negate()).whileTrue(clearKickerShooterCommand());

        zeroHoodTrigger.and(RobotModeTriggers.disabled().negate()).onTrue(zeroHoodCommand);
        zeroIntakeWristTrigger.and(RobotModeTriggers.disabled().negate()).onTrue(zeroIntakeWristCommand);

        runSysIdTrigger.whileTrue(sysId.getSysIdCommand());

        lockTrigger.whileTrue(driveSubsystem.lockPose());

        runIntakeTrigger
                .and(reverseIndexerTrigger.negate())
                .and(shootTrigger.negate())
                .whileTrue(runIndexerWithIntakeCommand);
        reverseIndexerTrigger.whileTrue(runIndexerReverseCommand);

        RobotModeTriggers.test().whileTrue(turretTestCommand);
        RobotModeTriggers.test().and(zeroHoodTrigger.negate()).whileTrue(hoodTestCommand);
        RobotModeTriggers.test().whileTrue(shooterTestCommand);
        RobotModeTriggers.test().and(zeroIntakeWristTrigger.negate()).whileTrue(testIntakeWristCommand);
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

    private Command clearKickerShooterCommand() {
        return Commands.parallel(
                shooter.run(() ->
                        shooter.runAtSpeed(MoPrefs.flywheelClearSpeed.get().unaryMinus())),
                kicker.run(
                        () -> kicker.runAtSpeed(MoPrefs.kickerClearSpeed.get().unaryMinus())));
    }
}
