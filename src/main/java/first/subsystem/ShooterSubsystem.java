package frc.robot.subsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.networktables.DoubleEntry;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.units.AngleUnit;
import org.wpilib.units.AngularVelocityUnit;
import org.wpilib.units.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.driverstation.XboxController;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.NTHelpers;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxPID;
import frc.robot.molib.pid.MoTalonFxPID.Type;
import frc.robot.shootutils.HoodSerializedInformationHolder;
import frc.robot.util.SysIdUtil;

public class ShooterSubsystem extends SubsystemBase {

    private final TalonFX motor1;
    private final TalonFX motor2;
    private final TalonFXConfiguration motor1Config;

    private final MoTalonFxPID<AngleUnit, AngularVelocityUnit> pid;
    private final MoRotationEncoder encoder;

    private final DoublePublisher flywheelCurrentPublisher;
    private final DoublePublisher flywheelSpeedPublisher;

    private final DoubleEntry flywheelTestSetpointEntry;

    private final DoublePublisher calculatedFlywheelSpeedPublisher;

    public ShooterSubsystem() {
        motor1 = new TalonFX(Constants.SHOOTER_1_ADDRESS.address());
        motor2 = new TalonFX(Constants.SHOOTER_2_ADDRESS.address());
        motor1Config = new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Coast)
                        .withInverted(InvertedValue.Clockwise_Positive))
                .withCurrentLimits(new CurrentLimitsConfigs()
                        .withStatorCurrentLimit((Current) MoPrefs.flywheelCurrentLimit.get())
                        .withStatorCurrentLimitEnable(true));
        motor1.getConfigurator().apply(motor1Config);

        MoPrefs.flywheelCurrentLimit.subscribe(current -> {
            motor1Config.CurrentLimits.withStatorCurrentLimit((Current) current);
            motor1.getConfigurator().apply(motor1Config);
        });

        motor2.setControl(new Follower(Constants.SHOOTER_1_ADDRESS.address(), MotorAlignmentValue.Opposed));

        encoder = MoRotationEncoder.forTalonFx(motor1, Units.Revolutions, motor1Config);
        pid = new MoTalonFxPID<>(Type.VELOCITY, motor1, encoder.getInternalEncoderUnits());

        TunerUtils.forMoTalonFx(pid, "Shooter PID");

        var table = NTHelpers.getTable("shooter-flywheel");
        flywheelCurrentPublisher = table.getDoubleTopic("Flywheel Current").publish();
        flywheelSpeedPublisher = table.getDoubleTopic("Flywheel Speed (RPM)").publish();

        flywheelTestSetpointEntry = NTHelpers.getDoubleEntry(table, "Flywheel Test Setpoint (RPM)", 500);

        calculatedFlywheelSpeedPublisher =
                table.getDoubleTopic("Calculated Flywheel Speed (RPM)").publish();
    }

    /**
     * Runs the motor at the speed indicated.
     * @param speed the speed to run the motor.
     */
    public void runAtSpeed(AngularVelocity velocity) {
        pid.setVelocityReference(velocity);
    }

    public void runAtCalculatedSpeed(Distance distanceToTarget) {
        AngularVelocity speed = HoodSerializedInformationHolder.getInstance().getFlywheelSpeed(distanceToTarget);
        calculatedFlywheelSpeedPublisher.set(speed.in(Units.RPM));
        runAtSpeed(speed);
    }

    public void stop() {
        motor1.stopMotor();
    }

    public boolean isUpToSpeed() {
        return pid.atSetpoint();
    }

    public double getMotorVelocity() {
        return encoder.getVelocityInEncoderUnits();
    }

    public SysIdRoutine.Mechanism getSysIdMechanism() {
        return SysIdUtil.sysIdMechanismForTalonFx(this, "shooter", motor1, encoder);
    }

    public Command getTestCommand(XboxController controller) {
        return run(() -> {
                    if (controller.getBButton()) {
                        runAtSpeed(Units.RPM.of(flywheelTestSetpointEntry.get()));
                    } else {
                        stop();
                    }
                })
                .withName("ShooterTestCommand");
    }

    @Override
    public void periodic() {
        flywheelCurrentPublisher.set(motor1.getStatorCurrent().getValueAsDouble());
        flywheelSpeedPublisher.set(encoder.getVelocity().in(Units.RPM));
    }
}
