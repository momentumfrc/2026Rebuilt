package frc.robot.commands.intake;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.MoPrefs;
import frc.robot.subsystem.IntakeWristSubsystem;
import java.util.function.Supplier;

public class MoveIntakeWristToPositionCommand extends Command {
    private final IntakeWristSubsystem wrist;
    private final Supplier<Angle> positionSupplier;

    private WristCommands.Direction direction;
    private Angle targetPos;

    private Timer timeout = new Timer();

    public MoveIntakeWristToPositionCommand(IntakeWristSubsystem wrist, Supplier<Angle> positionSupplier) {
        this.wrist = wrist;
        this.positionSupplier = positionSupplier;

        addRequirements(wrist);
    }

    @Override
    public void initialize() {
        targetPos = positionSupplier.get();
        var currPos = wrist.getPosition();
        if (targetPos.lt(currPos)) {
            direction = WristCommands.Direction.IN;
        } else {
            direction = WristCommands.Direction.OUT;
        }

        timeout.restart();
    }

    @Override
    public void execute() {
        var voltage = (Voltage)
                switch (direction) {
                    case IN -> MoPrefs.intakeWristVoltage.get().unaryMinus();
                    case OUT -> MoPrefs.intakeWristVoltage.get();
                };
        wrist.moveVoltage(voltage);
    }

    @Override
    public boolean isFinished() {
        if (timeout.hasElapsed(MoPrefs.intakeAgitateTimeout.get())) {
            return true;
        }

        return switch (direction) {
            case IN -> wrist.getPosition().lte(targetPos);
            case OUT -> wrist.getPosition().gte(targetPos);
        };
    }
}
