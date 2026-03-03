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
        return driveController.getLeftY();
    }

    @Override
    public double getDriveMoveYRequest() {
        return driveController.getLeftX();
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
        return operatorController.getYButton();
    }

    @Override
    public boolean getExtendIntake() {
        return operatorController.getLeftBumperButton();
    }

    @Override
    public boolean getRetractIntake() {
        return operatorController.getRightBumperButton();
    }

    @Override
    public boolean getShootRequest() {
        return operatorController.getAButton();
    }

    @Override
    public boolean getReverseIndexerRequest() {
        return operatorController.getXButton();
    }

    @Override
    public boolean getRunSysId() {
        return driveController.getRightBumperButton();
    }

    @Override
    public boolean getLockRequest() {
        return driveController.getXButton();
    }
}
