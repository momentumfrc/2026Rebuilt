package frc.robot.commands;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotPositioning;
import frc.robot.subsystem.IntakeRollerSubsystem;
import frc.robot.subsystem.LEDSubsystem;
import frc.robot.subsystem.TurretSubsystem;

public class LEDCommand extends Command {
    private final LEDSubsystem leds;
    private final RobotPositioning positioning;
    private final TurretSubsystem turret;
    private final IntakeRollerSubsystem rollerSubsystem;
    private final ShootCommand shootCommand;

    private static final LEDPattern inRangePattern =
            LEDPattern.solid(Color.kGreen).blink(Units.Seconds.of(0.1));

    private static final LEDPattern outOfRangePattern =
            LEDPattern.solid(Color.kRed).blink(Units.Seconds.of(0.5));

    private static final LEDPattern runIntakePattern = LEDPattern.solid(Color.kBlue).blink(Units.Seconds.of(0.5));

    private static final LEDPattern shootingPattern = LEDPattern.solid(Color.kOrange).blink(Units.Seconds.of(0.5));

    private static final LEDPattern noInitialPositionPattern = LEDPattern.solid(Color.kOrange);

    public LEDCommand(LEDSubsystem leds, RobotPositioning positioning, TurretSubsystem turret, IntakeRollerSubsystem rollerSubsystem, ShootCommand shootCommand) {
        this.leds = leds;
        this.positioning = positioning;
        this.turret = turret;
        this.rollerSubsystem = rollerSubsystem;
        this.shootCommand = shootCommand;

        addRequirements(leds);
    }

    @Override
    public void execute() {
        if (positioning.hasInitialPosition() == false) {
            leds.applyPattern(noInitialPositionPattern);
            return;
        }

        if (rollerSubsystem.runningRollers()) {
            leds.applyPattern(runIntakePattern);
        }

        if (shootCommand.currentlyShooting()) {
            leds.applyPattern(shootingPattern);
        }

        if (turret.targetInRange()) {
            leds.applyPattern(inRangePattern);
        } else {
            leds.applyPattern(outOfRangePattern);
        }

    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
