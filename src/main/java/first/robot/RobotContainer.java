// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import first.robot.commands.ShotBlockerCommand;
import first.robot.commands.ZeroShotBlockerCommand;
import first.robot.commands.auto.AutoChooser;
import first.robot.input.ControllerInput;
import first.robot.input.MoInput;
import first.robot.input.SingleControllerInput;
import first.robot.subsystem.DriveSubsystem;
import first.robot.subsystem.ShotBlockerSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;

public class RobotContainer {
    // **** SUBSYSTEMS ****
    private final DriveSubsystem driveSubsystem = new DriveSubsystem();
    private final ShotBlockerSubsystem shotBlockerSubsystem = new ShotBlockerSubsystem();

    // **** UTILITIES ****
    public final RobotPositioning robotPositioning = new RobotPositioning(driveSubsystem.getSwerveDrive());

    private final ControllerInput controllerInput = new ControllerInput();
    private final SingleControllerInput singleControllerInput = new SingleControllerInput();

    private final Selectable<MoInput> inputChooser = new Selectable<>();

    // **** COMMANDS ****
    private final Command driveCommand = driveSubsystem.getTeleopDriveCommand(this::getInput);

    private final Command zeroShotBlockerCommand = new ZeroShotBlockerCommand(shotBlockerSubsystem);
    private final Command shotBlockerDefaultCommand = new ShotBlockerCommand(shotBlockerSubsystem, this::getInput);

    // **** TRIGGERS ****
    private Trigger resetFieldOrientedFwd;

    private Trigger boostTrigger;

    private Trigger zeroShotBlockerTrigger;

    private TelemetryTable subsystemsTable = Telemetry.getTable("subsystems");

    private AutoChooser autochooser = new AutoChooser(robotPositioning, driveSubsystem);

    public RobotContainer() {
        addInputChooserToDashboard();
        configureBindings();
        setDefaultCommands();
    }

    public void setAutoDefaultCommnds() {
        driveSubsystem.setDefaultCommand(driveSubsystem.run(driveSubsystem::stop));
        shotBlockerSubsystem.setDefaultCommand(
                Commands.either(Commands.none(), zeroShotBlockerCommand, () -> shotBlockerSubsystem.hasZero()));
    }

    public void setDefaultCommands() {
        driveSubsystem.setDefaultCommand(driveCommand);
        shotBlockerSubsystem.setDefaultCommand(shotBlockerDefaultCommand);
    }

    private void addSubsystemsToDashboard() {
        subsystemsTable.log("driveSubsystem", driveSubsystem);
        subsystemsTable.log("shotBlockerSubsystem", shotBlockerSubsystem);
    }

    private void addInputChooserToDashboard() {
        inputChooser.addDefault("Driver + Operator", controllerInput);
        inputChooser.add("Single Driver", singleControllerInput);
        Tunables.publish("Controller Input Chooser", inputChooser);
    }

    private void configureBindings() {
        // Drive Triggers
        resetFieldOrientedFwd = new Trigger(() -> getInput().getReZeroGyro());

        boostTrigger = new Trigger(() -> getInput().getDriveBoostRequest());

        zeroShotBlockerTrigger = new Trigger(() -> shotBlockerSubsystem.hasZero() == false);

        // Drive Trigger Bindings
        resetFieldOrientedFwd.onTrue(driveSubsystem.resetFieldOrientedFwd());

        boostTrigger.onTrue(
                Commands.runOnce(driveSubsystem::toggleBoostCurrentLimits).ignoringDisable(true));

        zeroShotBlockerTrigger.and(RobotModeTriggers.teleop()).onTrue(zeroShotBlockerCommand);
    }

    public Command getAutonomousCommand() {
        return autochooser.getAutoChooserCommand();
    }

    private MoInput getInput() {
        return inputChooser.getSelected();
    }

    public void checkRumbles() {
        double rumble = driveSubsystem.isBoosted() ? MoPrefs.driverRumble.get() : 0;
        controllerInput.getDriveController().setRumble(RumbleType.LEFT_RUMBLE, rumble);
    }

    public void telemetry() {
        addSubsystemsToDashboard();
    }
}
