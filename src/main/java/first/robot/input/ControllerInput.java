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
    public double getShotBlockerRequest() {
        return (-1 * driveController.getRightTrigger()) + driveController.getLeftTrigger();
    }
}
