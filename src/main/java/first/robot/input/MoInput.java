package first.robot.input;

public interface MoInput {
    public abstract double getDriveMoveXRequest();

    public abstract double getDriveMoveYRequest();

    public abstract double getDriveTurnRequest();

    public abstract boolean getDriveBoostRequest();

    public abstract double getDriveHeadingXRequest();

    public abstract double getDriveHeadingYRequest();

    public abstract boolean getReZeroGyro();

    public abstract boolean getRunIntake();

    public abstract boolean getRunIntakeReverse();

    public abstract boolean getClearShooter();

    public abstract boolean getAgitate();

    public abstract boolean getExtendIntake();

    public abstract boolean getRetractIntake();

    public abstract boolean getShootRequest();

    public abstract boolean getShuttleRequest();

    public abstract boolean getReverseIndexerRequest();

    public abstract boolean getRunSysId();

    public abstract boolean getLockRequest();
}
