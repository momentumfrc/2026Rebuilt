package first.robot.subsystem;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import first.robot.Constants;
import first.robot.MoPrefs;
import first.robot.molib.NTHelpers;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.units.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class ShotBlockerSubsystem extends SubsystemBase {

    private final SparkFlex winch;
    private final RelativeEncoder winchEncoder;

    private final BooleanEntry hasZero;

    private final SparkFlexConfig config;

    private final DoublePublisher positionPublisher;
    private final DoublePublisher velocityPublisher;
    private final DoublePublisher currentPublisher;

    public ShotBlockerSubsystem() {

        winch = new SparkFlex(
                Constants.DEFAULT_CAN_BUS, Constants.SHOT_BLOCKER_ADDRESS.address(), MotorType.kBrushless);
        winchEncoder = winch.getEncoder();

        config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake)
                .smartCurrentLimit((int) MoPrefs.shotBlockerCurrentLimit.get().in(Units.Amps));
        config.softLimit
                .reverseSoftLimit(0)
                .reverseSoftLimitEnabled(true)
                .forwardSoftLimit(MoPrefs.shotBlockerMaxHeight.get())
                .forwardSoftLimitEnabled(true);
        winch.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        MoPrefs.shotBlockerCurrentLimit.subscribe(limit -> {
            config.smartCurrentLimit((int) limit.in(Units.Amps));
            winch.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        });
        MoPrefs.shotBlockerMaxHeight.subscribe(height -> {
            config.softLimit.forwardSoftLimit(height);
            winch.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        });

        NetworkTable table = NTHelpers.getTable("shot-blocker");
        hasZero = NTHelpers.getBooleanEntry(table, "Has zero?", false);

        positionPublisher = table.getDoubleTopic("SB Position").publish();
        velocityPublisher = table.getDoubleTopic("SB Velocity (RPM)").publish();
        currentPublisher = table.getDoubleTopic("SB Current (A)").publish();
    }

    public void runWinch(Voltage power) {
        winch.setVoltage(power);
    }

    public void stop() {
        winch.setVoltage(0);
    }

    public boolean hasZero() {
        return hasZero.get();
    }

    public void invalidateZero() {
        hasZero.set(false);
    }

    public void setZero(double pos) {
        hasZero.set(true);
        winchEncoder.setPosition(pos);
    }

    public void setEnableWinchReverseSoftLimit(boolean enable) {
        config.softLimit.reverseSoftLimitEnabled(enable);
        winch.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    public Current getWinchMotorCurrent() {
        return Units.Amps.of(winch.getOutputCurrent().get());
    }

    public AngularVelocity getWinchMotorVelocity() {
        return Units.RPM.of(winchEncoder.getVelocity().get());
    }

    @Override
    public void periodic() {
        positionPublisher.set(winchEncoder.getPosition().get());
        velocityPublisher.set(winchEncoder.getVelocity().get());
        currentPublisher.set(winch.getOutputCurrent().get());
    }
}
