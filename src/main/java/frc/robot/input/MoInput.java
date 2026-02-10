package frc.robot.input;

public interface MoInput {
    public abstract double getDriveMoveXRequest();

    public abstract double getDriveMoveYRequest();

    public abstract double getDriveTurnRequest();

    public abstract boolean getReZeroGyro();

    public abstract boolean getRunIntake();

    public abstract boolean getExtendIntake();

    public abstract boolean getRetractIntake();

}
