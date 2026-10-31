package first.robot.input;

import first.robot.Constants;
import org.wpilib.driverstation.NiDsXboxController;

public final class ControllerInput implements MoInput {
    private final NiDsXboxController driveController = new NiDsXboxController(Constants.DRIVE_CONTORLLER_PORT.hidport());
    private final NiDsXboxController operatorController = new NiDsXboxController(Constants.OPERATOR_CONTROLLER_PORT.hidport());

    public NiDsXboxController getDriveController() {
        return driveController;
    }

    public NiDsXboxController getOperatorController() {
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
    public boolean getRetractShotBlocker() {
        return driveController.getLeftTriggerAxis() > 0.5;
    }
}
