package frc.robot.input;

import java.util.function.Supplier;

public class InputTransformer implements MoInput{
    private final Supplier<MoInput> inputSupplier;

    public InputTransformer(Supplier<MoInput> inputSupplier) {
        this.inputSupplier = inputSupplier;
    }

     @Override
    public double getDriveMoveXRequest() {
        return inputSupplier.get().getDriveMoveXRequest();
    }

    @Override
    public double getDriveMoveYRequest() {
        return inputSupplier.get().getDriveMoveYRequest();
    }

    @Override
    public double getDriveTurnRequest() {
        return inputSupplier.get().getDriveTurnRequest();
    }

    @Override
    public double getDriveHeadingXRequest() {
        return inputSupplier.get().getDriveHeadingXRequest();
    }

    @Override
    public double getDriveHeadingYRequest() {
        return inputSupplier.get().getDriveHeadingYRequest();
    }

    @Override
    public boolean getReZeroGyro() {
        return inputSupplier.get().getReZeroGyro();
    }

    @Override
    public boolean getRunIntake() {
        return inputSupplier.get().getRunIntake();
    }

    @Override
    public boolean getAgitate() {
        return inputSupplier.get().getAgitate();
    }

    @Override
    public boolean getClearShooter() {
        return inputSupplier.get().getClearShooter();
    }

    @Override
    public boolean getExtendIntake() {
        return inputSupplier.get().getExtendIntake();
    }

    @Override
    public boolean getRetractIntake() {
        return inputSupplier.get().getRetractIntake();
    }

    @Override
    public boolean getShootRequest() {
        return inputSupplier.get().getShootRequest();
    }

    @Override
    public boolean getReverseIndexerRequest() {
        return inputSupplier.get().getReverseIndexerRequest();
    }

    @Override
    public boolean getRunSysId() {
        return inputSupplier.get().getRunSysId();
    }

    @Override
    public boolean getLockRequest() {
        return inputSupplier.get().getLockRequest();
    }
}
