package frc.robot.subsystem;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MoPrefs;

public class IndexerSubsystem extends SubsystemBase {

    private final TalonFX motor;

    public IndexerSubsystem() {
        motor = new TalonFX(Constants.INDEXER_PORT.address());

        // to make our lives easier
        motor.setNeutralMode(NeutralModeValue.Coast);
    }

    /**
     * Runs the motor at the given speed.
     * @param speed speed to run motor at, from [-1.0, 1.0]
     */
    public void run(double speed) {
        motor.set(speed);
    }

    public void run() {
        run(MoPrefs.indexerRunPercentage.get().in(Units.Value));
    }

    public void stop() {
        motor.stopMotor();
    }
}
