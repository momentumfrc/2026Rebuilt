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
    public boolean getDriveBoostRequest() {
        return false;
    }

    @Override
    public boolean getRetractShotBlocker() {
        return driveController.getLeftBumperButton();
    }
}
