package frc.robot.commands;

import org.wpilib.units.Units;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.util.Color;
import org.wpilib.command2.Command;
import frc.robot.RobotPositioning;
import frc.robot.subsystem.LEDSubsystem;
import frc.robot.subsystem.TurretSubsystem;

public class LEDCommand extends Command {
    private final LEDSubsystem leds;
    private final RobotPositioning positioning;
    private final TurretSubsystem turret;

    private static final LEDPattern inRangePattern =
            LEDPattern.solid(Color.kGreen).blink(Units.Seconds.of(0.1));

    private static final LEDPattern outOfRangePattern =
            LEDPattern.solid(Color.kRed).blink(Units.Seconds.of(0.5));

    private static final LEDPattern noInitialPositionPattern = LEDPattern.solid(Color.kOrange);

    public LEDCommand(LEDSubsystem leds, RobotPositioning positioning, TurretSubsystem turret) {
        this.leds = leds;
        this.positioning = positioning;
        this.turret = turret;

        addRequirements(leds);
    }

    @Override
    public void execute() {
        if (positioning.hasInitialPosition() == false) {
            leds.applyPattern(noInitialPositionPattern);
            return;
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
