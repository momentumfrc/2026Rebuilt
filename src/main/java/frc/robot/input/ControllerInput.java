package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class ControllerInput implements MoInput {
    private final XboxController driveController = new XboxController(Constants.DRIVE_CONTORLLER_PORT.hidport());

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
    public boolean getReZeroGyro() {
        return driveController.getBackButton();
    }

    @Override
    public boolean getShootRequest() {
        return driveController.getAButton();
    }
}
