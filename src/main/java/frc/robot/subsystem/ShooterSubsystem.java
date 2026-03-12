package frc.robot.subsystem;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.MoPrefs;
import frc.robot.molib.encoder.MoRotationEncoder;
import frc.robot.molib.motune.TunerUtils;
import frc.robot.molib.pid.MoTalonFxPID;
import frc.robot.molib.pid.MoTalonFxPID.Type;
import frc.robot.shootutils.HoodSerializedInformationHolder;
import frc.robot.util.NTHelpers;
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

        flywheelTestSetpointEntry =
                table.getDoubleTopic("Flywheel Test Setpoint").getEntry(500);

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
        return Commands.startRun(
                        () -> flywheelSpeedPublisher.set(500),
                        () -> {
                            if (controller.getRightBumperButton()) {
                                runAtSpeed(Units.RPM.of(flywheelTestSetpointEntry.get()));
                            } else {
                                stop();
                            }
                        },
                        this)
                .withName("ShooterTestCommand");
    }

    @Override
    public void periodic() {
        flywheelCurrentPublisher.set(motor1.getStatorCurrent().getValueAsDouble());
        flywheelSpeedPublisher.set(encoder.getVelocity().in(Units.RPM));
    }
}
