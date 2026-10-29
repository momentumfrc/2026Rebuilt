package first.robot.util;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import first.robot.MoPrefs;
import first.robot.molib.NTHelpers;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class AutomaticZeroArm {

    /* A single side climber */
    private static final Current CLIMBER_CURRENT_LIMIT = Units.Amps.of(25);

    public final SparkFlex winch;
    private final RelativeEncoder encoder;

    private final BooleanEntry hasZero;

    private final SparkFlexConfig sparkMaxConfig;

    private final DoublePublisher positionPublisher;
    private final DoublePublisher currentPublisher;

    public AutomaticZeroArm(String name, SparkFlex winch, boolean inverted) {
        this.winch = winch;

        this.encoder = winch.getEncoder();

        sparkMaxConfig = new SparkFlexConfig();
        sparkMaxConfig.smartCurrentLimit((int) CLIMBER_CURRENT_LIMIT.in(Units.Amps));
        sparkMaxConfig.idleMode(IdleMode.kBrake);

        final SoftLimitConfig softLimitConfig = new SoftLimitConfig();
        softLimitConfig.reverseSoftLimit(0);

        sparkMaxConfig.softLimit.apply(softLimitConfig);

        MoPrefs.shotBlockerMaxHeight.subscribe(
                max -> {
                    sparkMaxConfig.softLimit.forwardSoftLimit(max);
                    sparkMaxConfig.softLimit.reverseSoftLimit(0);
                    winch.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
                },
                true);

        sparkMaxConfig.softLimit.forwardSoftLimitEnabled(true);
        sparkMaxConfig.softLimit.reverseSoftLimitEnabled(true);

        winch.configure(sparkMaxConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        NetworkTable table = NTHelpers.getTable("shot-blocker");

        hasZero = NTHelpers.getBooleanEntry(table, "Has zero?", false);

        positionPublisher = table.getDoubleTopic("Position (cm)").publish();
        currentPublisher = table.getDoubleTopic("Current (A)").publish();
    }

    public boolean hasZero() {
        return hasZero.get();
    }

    public void invalidateZero() {
        hasZero.set(false);
    }

    public void setZero(double pos) {
        hasZero.set(true);
        this.encoder.setPosition(pos);
    }

    public void runWinch(Voltage power) {
        this.winch.setVoltage(power);
    }

    public void enableWinchSoftLimitReverse(boolean enable) {
        sparkMaxConfig.softLimit.reverseSoftLimitEnabled(enable);
        winch.configure(sparkMaxConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public void stop() {
        this.winch.stopMotor();
    }

    public void publish() {
        positionPublisher.set(encoder.getPosition().get());
        currentPublisher.set(winch.getOutputCurrent().get());
    }
}
