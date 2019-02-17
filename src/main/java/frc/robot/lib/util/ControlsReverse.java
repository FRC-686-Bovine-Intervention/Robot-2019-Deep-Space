package frc.robot.lib.util;

import frc.robot.Constants;
import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;

public class ControlsReverse {
    public static ControlsReverse mInstance = new ControlsReverse();

    public static ControlsReverse getInstance() {
        return mInstance;
    }

    public double lSpeed; // from joystick
    public double rSpeed;

    public double newL; // given
    public double newR;
    public RisingEdgeDetector risingEdgeDetector = new RisingEdgeDetector();

    public DriveCommand run(DriveCommand driveCmd, int ReverseButton) {
        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
        boolean btnIsPushed = controls.getButton(Constants.kControlsReverseButton);
        boolean edgeDetectorValue = risingEdgeDetector.update(btnIsPushed);

        lSpeed = driveCmd.getLeftMotor(); // lSpeed overidden
        rSpeed = driveCmd.getRightMotor();
        newL = lSpeed;
        newR = rSpeed;

        if (edgeDetectorValue) {
            newL = -lSpeed;
            newR = -rSpeed;
        }

        return new DriveCommand(newL, newR);
    }
}
