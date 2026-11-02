package first.robot.input;

import first.robot.Constants;
import org.wpilib.driverstation.NiDsXboxController;

public class SingleControllerInput implements MoInput {
    private final NiDsXboxController driveController =
            new NiDsXboxController(Constants.DRIVE_CONTORLLER_PORT.hidport());

    public NiDsXboxController getDriveController() {
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
        return driveController.getAButton();
    }

    @Override
    public boolean getDriveLock() {
        return driveController.getXButton();
    }


    @Override
    public boolean getMoveShotBlocker() {
        return driveController.getLeftTriggerAxis() > 0.5 || driveController.getRightTriggerAxis() > 0.5;
    }

}
