package first.robot.input;

import first.robot.Constants;
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
        return driveController.getFaceRightButton();
    }

    @Override
    public boolean getExtendIntake() {
        return driveController.getLeftTrigger() > 0;
    }

    @Override
    public boolean getRetractIntake() {
        return driveController.getRightTrigger() > 0;
    }

    @Override
    public boolean getShootRequest() {
        return driveController.getFaceDownButton();
    }

    @Override
    public boolean getReverseIndexerRequest() {
        return driveController.getFaceUpButton();
    }

    // use operator controller if you need this
    @Override
    public boolean getRunSysId() {
        return false;
    }

    @Override
    public boolean getLockRequest() {
        return driveController.getFaceLeftButton();
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
