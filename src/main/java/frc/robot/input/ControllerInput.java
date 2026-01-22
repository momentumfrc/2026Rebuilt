package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class ControllerInput implements MoInput {
    private final XboxController driveController = new XboxController(Constants.DRIVE_CONTORLLER_PORT.hidport());

    @Override
    public double getDriveMoveXRequest() {
        return -1 * driveController.getLeftX();
    }

    @Override
    public double getDriveMoveYRequest() {
        return -1 * driveController.getLeftY();
    }

    @Override
    public double getDriveTurnRequest() {
        return -1 * driveController.getRightX();
    }

    @Override
    public boolean getReZeroGyro() {
        return driveController.getBackButton();
    }
}
