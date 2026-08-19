package first.robot.commands;

import first.robot.RobotPositioning;
import first.robot.subsystem.LEDSubsystem;
import first.robot.subsystem.TurretSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.units.Units;
import org.wpilib.util.Color;

public class LEDCommand extends Command {
    private final LEDSubsystem leds;
    private final RobotPositioning positioning;
    private final TurretSubsystem turret;

    private static final LEDPattern inRangePattern =
            LEDPattern.solid(Color.GREEN).blink(Units.Seconds.of(0.1));

    private static final LEDPattern outOfRangePattern =
            LEDPattern.solid(Color.RED).blink(Units.Seconds.of(0.5));

    private static final LEDPattern noInitialPositionPattern = LEDPattern.solid(Color.ORANGE);

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
