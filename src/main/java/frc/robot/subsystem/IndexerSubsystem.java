package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.MoTuner;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxPID;
import frc.robot.molib.pid.MoTalonFxPID.Type;
import frc.robot.molib.prefs.MoPrefsUtils;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX motor;

    private final MoRotationEncoder encoder;

    private final MoTalonFxPID<AngleUnit, AngularVelocityUnit> pid;

    public IndexerSubsystem() {

        motor = new TalonFX(Constants.INDEXER_PORT.address());

        encoder = MoRotationEncoder.forTalonFx(motor, Units.Revolutions);
        MoPrefs.indexerEncoderScale.subscribe(encoder::setConversionFactor, true);

        pid = new MoTalonFxPID<>(Type.VELOCITY, motor, encoder.getInternalEncoderUnits());

        TunerUtils.forMoTalonFx(pid, "Indexer PID");

        // to make things easier...
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    public void run() {
        pid.setVelocityReference(MoPrefs.indexerRunSpeed.get());
    }

    public void stop() {
        pid.setVelocityReference(Units.RevolutionsPerSecond.of(0));
        ;
    }
}
