package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public final class ControllerInput implements MoInput {
    private final XboxController driveController = new XboxController(Constants.DRIVE_CONTORLLER_PORT.hidport());
    private final XboxController operatorController = new XboxController(Constants.OPERATOR_CONTROLLER_PORT.hidport());

    public XboxController getDriveController() {
        return driveController;
    }

    public XboxController getOperatorController() {
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
        return driveController.getAButton();
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
        return operatorController.getRightBumperButton();
    }

    @Override
    public boolean getRunIntakeReverse() {
        return operatorController.getRightTriggerAxis() > 0.8;
    }

    @Override
    public boolean getAgitate() {
        return operatorController.getLeftBumperButton();
    }

    @Override
    public boolean getClearShooter() {
        return operatorController.getBButton();
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
        return operatorController.getAButton();
    }

    @Override
    public boolean getShuttleRequest() {
        return operatorController.getYButton();
    }

    @Override
    public boolean getReverseIndexerRequest() {
        return operatorController.getXButton();
    }

    @Override
    public boolean getRunSysId() {
        return driveController.getStartButton();
    }

    @Override
    public boolean getLockRequest() {
        return driveController.getXButton();
    }
}
