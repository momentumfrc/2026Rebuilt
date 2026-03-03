package frc.robot.subsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxPID;
import frc.robot.molib.pid.MoTalonFxPID.Type;
import frc.robot.util.NTHelpers;
import frc.robot.util.SysIdUtil;

@Logged
public class KickerSubsystem extends SubsystemBase {

    private final TalonFX motor;
    private final TalonFXConfiguration config;

    private final MoRotationEncoder encoder;

    private final MoTalonFxPID<AngleUnit, AngularVelocityUnit> pid;

    private final DoublePublisher speedPublisher;

    public KickerSubsystem() {
        motor = new TalonFX(Constants.KICKER_PORT.address());
        config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit((Current) MoPrefs.kickerCurrentLimit.get())
                        .withStatorCurrentLimitEnable(true));
        motor.getConfigurator().apply(config);

        encoder = MoRotationEncoder.forTalonFx(motor, Units.Revolutions);
        MoPrefs.kickerEncoderScale.subscribe(encoder::setConversionFactor, true);
        MoPrefs.kickerCurrentLimit.subscribe(limit -> {
            config.CurrentLimits.withStatorCurrentLimit((Current) limit);
            motor.getConfigurator().apply(config);
        });

        pid = new MoTalonFxPID<>(Type.VELOCITY, motor, encoder.getInternalEncoderUnits());

        TunerUtils.forMoTalonFx(pid, "Kicker PID");

        NetworkTable table = NTHelpers.getTable("kicker");

        speedPublisher = table.getDoubleTopic("Kicker Speed (RPM)").publish();
    }

    public void run() {
        pid.setVelocityReference(MoPrefs.kickerRunSpeed.get());
    }

    public void stop() {
        pid.setVelocityReference(Units.RevolutionsPerSecond.zero());
    }

    public SysIdRoutine.Mechanism getSysIdMechanism() {
        return SysIdUtil.sysIdMechanismForTalonFx(this, "kicker", motor, encoder);
    }

    @Override
    public void periodic() {
        speedPublisher.set(encoder.getVelocity().in(Units.RPM));
    }
}
