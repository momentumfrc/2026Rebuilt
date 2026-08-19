package first.input;

import first.Constants;

import org.wpilib.driverstation.Gamepad;

public class SingleControllerInput implements MoInput {
    private final Gamepad driveController = new Gamepad(Constants.DRIVE_CONTORLLER_PORT.hidport());

    public Gamepad getDriveController() {
        return driveController;
    }

    @Override
    public double getDriveMoveXRequest() {
        return -1 * driveController.getLeftY();
    }

    @Override
    public double getDriveMoveYRequest() {
        return -1 * driveController.getLeftX();
    }

    @Override
    public double getDriveTurnRequest() {
        return -1 * driveController.getRightX();
    }

    @Override
    public double getDriveHeadingXRequest() {
        return driveController.getRightY();
    }

    @Override
    public double getDriveHeadingYRequest() {
        return driveController.getRightX();
    }

    @Override
    public boolean getReZeroGyro() {
        return driveController.getBackButton();
    }

    @Override
    public boolean getRunIntake() {
        return driveController.getRightBumperButton();
    }

    @Override
    public boolean getAgitate() {
        return driveController.getLeftBumperButton();
    }

    @Override
    public boolean getClearShooter() {
        return driveController.getEastFaceButton();
    }

    @Override
    public boolean getExtendIntake() {
        return driveController.getLeftTriggerAxis() > 0;
    }

    @Override
    public boolean getRetractIntake() {
        return driveController.getRightTriggerAxis() > 0;
    }

    @Override
    public boolean getShootRequest() {
        return driveController.getSouthFaceButton();
    }

    @Override
    public boolean getReverseIndexerRequest() {
        return driveController.getNorthFaceButton();
    }

    // use operator controller if you need this
    @Override
    public boolean getRunSysId() {
        return false;
    }

    @Override
    public boolean getLockRequest() {
        return driveController.getWestFaceButton();
    }

    @Override
    public boolean getShuttleRequest() {
        return false;
    }

    @Override
    public boolean getRunIntakeReverse() {
        return false;
    }

    @Override
    public boolean getDriveBoostRequest() {
        return false;
    }
}
