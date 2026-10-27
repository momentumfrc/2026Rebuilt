package first.robot.subsystem;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants;
import first.robot.util.AutomaticZeroArm;
import org.wpilib.command2.SubsystemBase;

public class ShotBlockerSubsystem extends SubsystemBase {

    public final AutomaticZeroArm arm;

    private final SparkMax motor;

    public ShotBlockerSubsystem() {

        motor = new SparkMax(Constants.DEFAULT_CAN_BUS, Constants.SHOT_BLOCKER_ADDRESS.address(), MotorType.kBrushless);

        arm = new AutomaticZeroArm(getName(), motor, false);
    }

    public void stop() {
        arm.stop();
    }

    public boolean zeroed() {
        return arm.hasZero();
    }
}
