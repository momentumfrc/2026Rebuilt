package first.robot.input;

public interface MoInput {
    public abstract double getDriveMoveXRequest();

    public abstract double getDriveMoveYRequest();

    public abstract double getDriveTurnRequest();

    public abstract boolean getDriveBoostRequest();

    public abstract double getDriveHeadingXRequest();

    public abstract double getDriveHeadingYRequest();

    public abstract boolean getReZeroGyro();

    public abstract boolean getRetractShotBlocker();
}
