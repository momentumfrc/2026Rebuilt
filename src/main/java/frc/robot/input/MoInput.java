package frc.robot.input;

public interface MoInput {
    public abstract double getDriveMoveXRequest();

    public abstract double getDriveMoveYRequest();

    public abstract double getDriveTurnRequest();

    public abstract boolean getReZeroGyro();
}
