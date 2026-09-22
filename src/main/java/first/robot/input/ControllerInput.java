package first.robot.input;

import first.robot.Constants;
import org.wpilib.driverstation.Gamepad;

public final class ControllerInput implements MoInput {
    private final Gamepad driveController = new Gamepad(Constants.DRIVE_CONTORLLER_PORT.hidport());
    private final Gamepad operatorController = new Gamepad(Constants.OPERATOR_CONTROLLER_PORT.hidport());

    public Gamepad getDriveController() {
        return driveController;
    }

    public Gamepad getOperatorController() {
        return operatorController;
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
    public boolean getDriveBoostRequest() {
        return driveController.getFaceDownButton();
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
        return operatorController.getRightBumperButton() || driveController.getRightTrigger() > 0.8;
    }

    @Override
    public boolean getRunIntakeReverse() {
        return operatorController.getRightTrigger() > 0.8;
    }

    @Override
    public boolean getAgitate() {
        return operatorController.getLeftBumperButton();
    }

    @Override
    public boolean getClearShooter() {
        return operatorController.getFaceRightButton();
    }

    @Override
    public boolean getExtendIntake() {
        return driveController.getLeftBumperButton();
    }

    @Override
    public boolean getRetractIntake() {
        return driveController.getRightBumperButton();
    }

    @Override
    public boolean getShootRequest() {
        return operatorController.getFaceDownButton();
    }

    @Override
    public boolean getShuttleRequest() {
        return operatorController.getFaceUpButton();
    }

    @Override
    public boolean getReverseIndexerRequest() {
        return operatorController.getFaceLeftButton();
    }

    @Override
    public boolean getRunSysId() {
        return driveController.getStartButton();
    }

    @Override
    public boolean getLockRequest() {
        return driveController.getFaceLeftButton();
    }
}
