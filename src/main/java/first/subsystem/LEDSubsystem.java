package frc.robot.subsystem;

import frc.robot.Constants;
import frc.robot.MoPrefs;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.led.AddressableLED;
import org.wpilib.hardware.led.AddressableLEDBuffer;
import org.wpilib.hardware.led.LEDPattern;
import org.wpilib.hardware.led.LEDWriter;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Distance;

public class LEDSubsystem extends SubsystemBase {
    private static final int LED_COUNT = 120;
    public static final Distance LED_SPACING = Units.Meters.of(1 / 60.0);

    private final AddressableLED leds;
    private final AddressableLEDBuffer ledBuffer;

    private final LEDWriter dimmer;

    public LEDSubsystem() {
        this.leds = new AddressableLED(Constants.LED_PORT.port());
        this.ledBuffer = new AddressableLEDBuffer(LED_COUNT);

        leds.setLength(LED_COUNT);
        leds.setData(ledBuffer);
        leds.start();

        dimmer = (i, r, g, b) -> {
            double k = MoPrefs.ledBrightness.get().in(Units.Value);
            ledBuffer.setRGB(i, (int) MathUtil.clamp(r * k, 0, 255), (int) MathUtil.clamp(g * k, 0, 255), (int)
                    MathUtil.clamp(b * k, 0, 255));
        };
    }

    public void applyPattern(LEDPattern pattern) {
        pattern.applyTo(ledBuffer, dimmer);
    }

    @Override
    public void periodic() {
        leds.setData(ledBuffer);
    }
}
