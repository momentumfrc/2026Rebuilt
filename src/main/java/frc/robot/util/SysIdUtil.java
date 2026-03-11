package frc.robot.util;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.molib.encoder.MoRotationEncoder;
import java.util.Collection;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

public class SysIdUtil {
    private enum SysIdMode {
        DISABLED,
        QUASISTATIC_FWD,
        QUASISTATIC_REV,
        DYNAMIC_FWD,
        DYNAMIC_REV
    };

    private final SendableChooser<SysIdMode> modeChooser = NTHelpers.enumToChooser(SysIdMode.class, SysIdMode.DISABLED);
    private final SendableChooser<SysIdRoutine.Mechanism> mechanismChooser = new SendableChooser<>();

    private final NetworkTable table;
    private final DoubleEntry rampRate;
    private final DoubleEntry stepVoltage;
    private final DoubleEntry timeout;

    private final Map<String, SysIdRoutine> routineCache = new HashMap<>();

    public SysIdUtil(Collection<SysIdRoutine.Mechanism> mechanisms) {
        assert mechanisms.isEmpty() == false;

        boolean first = true;
        for (var mechanism : mechanisms) {
            if (first) {
                mechanismChooser.setDefaultOption(mechanism.m_name, mechanism);
                first = false;
            } else {
                mechanismChooser.addOption(mechanism.m_name, mechanism);
            }
        }

        table = NTHelpers.getTable("sysid");

        rampRate = table.getDoubleTopic("ramp rate (v_s)").getEntry(2);
        rampRate.set(2);

        stepVoltage = table.getDoubleTopic("step voltage (v)").getEntry(3);
        stepVoltage.set(3);

        timeout = table.getDoubleTopic("timeout (s)").getEntry(5);
        timeout.set(5);

        table.addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {
            routineCache.clear();
        });

        NTHelpers.publishSendable(table, "SysId Mode", modeChooser);
        NTHelpers.publishSendable(table, "SysId Mechanism", mechanismChooser);
    }

    private SysIdRoutine.Config getSysIdConfig() {
        return new SysIdRoutine.Config(
                Units.Volts.per(Units.Second).of(rampRate.get()),
                Units.Volts.of(stepVoltage.get()),
                Units.Seconds.of(timeout.get()));
    }

    public Command getSysIdCommand() {
        return Commands.deferredProxy(() -> {
            var mechanism = mechanismChooser.getSelected();
            var routine = routineCache.computeIfAbsent(
                    mechanism.m_name, (name) -> new SysIdRoutine(getSysIdConfig(), mechanism));
            return switch (modeChooser.getSelected()) {
                case DISABLED -> Commands.print("SysId is disabled");
                case QUASISTATIC_FWD -> routine.quasistatic(SysIdRoutine.Direction.kForward);
                case QUASISTATIC_REV -> routine.quasistatic(SysIdRoutine.Direction.kReverse);
                case DYNAMIC_FWD -> routine.dynamic(SysIdRoutine.Direction.kForward);
                case DYNAMIC_REV -> routine.dynamic(SysIdRoutine.Direction.kReverse);
            };
        });
    }

    public static SysIdRoutine.Mechanism sysIdMechanismForTalonFx(
            Subsystem subsystem, String motorName, TalonFX talon, MoRotationEncoder encoder) {
        final AngularAccelerationUnit encoderOmegaUnits =
                encoder.getInternalEncoderUnits().per(Units.Second).per(Units.Second);
        return new SysIdRoutine.Mechanism(
                v -> {
                    var request = new VoltageOut(v);
                    talon.setControl(request);
                    System.out.println(request.Output);
                },
                log -> log.motor(motorName)
                        .voltage(talon.getMotorVoltage().getValue())
                        .value(
                                "position",
                                encoder.getPositionInEncoderUnits(),
                                encoder.getInternalEncoderUnits().name())
                        .value(
                                "velocity",
                                encoder.getVelocityInEncoderUnits(),
                                encoder.getInternalEncoderVelocityUnits().name())
                        .value("acceleration", talon.getAcceleration().getValueAsDouble(), encoderOmegaUnits.name()),
                subsystem);
    }
}
