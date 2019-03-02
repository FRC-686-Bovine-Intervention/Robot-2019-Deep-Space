package frc.robot.lib.joystick;

import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.lib.util.Util;
import frc.robot.command_status.DriveCommand;

/**
 * Implements a simple arcade drive, where single stick is used for throttle and
 * turn.
 */
public class ArcadeDriveJoystick extends JoystickControlsBase {
	// singleton class
	private static JoystickControlsBase instance = null;

	public static JoystickControlsBase getInstance() {
		if (instance == null) {
			instance = new ArcadeDriveJoystick();
		}
		return instance;
	}

	static double kDeadband = 0.05;

	public DriveCommand getDriveCommand() {
		double throttle = -mStick.getY(); 
		double turn = -mStick.getX(); 

		DriveCommand signal = throttleTurnToDriveCommand(throttle, turn);

		return signal;
	}

	public static DriveCommand throttleTurnToDriveCommand(double throttle, double turn) {
		boolean squaredInputs = true; // set to true to increase fine control while permitting full power

		if (throttle < kJoystickDeadzone && throttle > -kJoystickDeadzone) {
			throttle = 0;
		}
		if (turn < kJoystickDeadzone && turn > -kJoystickDeadzone) {
			turn = 0;
		}

		double moveValue = applyDeadband(throttle);
		double rotateValue = applyDeadband(turn);
		double lMotorSpeed, rMotorSpeed;

		// if (squaredInputs) {
		// 	// square the inputs (while preserving the sign) to increase fine control
		// 	// while permitting full power
		// 	if (moveValue >= 0.0) {
		// 		moveValue = (moveValue * moveValue);
		// 	} else {
		// 		moveValue = -(moveValue * moveValue);
		// 	}
		// 	if (rotateValue >= 0.0) {
		// 		rotateValue = (rotateValue * rotateValue);
		// 	} else {
		// 		rotateValue = -(rotateValue * rotateValue);
		// 	}
		// }
		if (squaredInputs) {
			// square the inputs (while preserving the sign) to increase fine control
			// while permitting full power
			if (moveValue >= 0.0) {
				moveValue = (moveValue * moveValue*moveValue);
			} else {
				moveValue = (moveValue * moveValue*moveValue);
			}
			if (rotateValue >= 0.0) {
				rotateValue = (rotateValue * rotateValue*rotateValue);
			} else {
				rotateValue = (rotateValue * rotateValue*rotateValue);
			}
		}


		if (moveValue > 0.0) {
			if (rotateValue > 0.0) {
				lMotorSpeed = moveValue - rotateValue;
				rMotorSpeed = Math.max(moveValue, rotateValue);
			} else {
				lMotorSpeed = Math.max(moveValue, -rotateValue);
				rMotorSpeed = moveValue + rotateValue;
			}
		} else {
			if (rotateValue > 0.0) {
				lMotorSpeed = -Math.max(-moveValue, rotateValue);
				rMotorSpeed = moveValue + rotateValue;
			} else {
				lMotorSpeed = moveValue - rotateValue;
				rMotorSpeed = -Math.max(-moveValue, -rotateValue);
			}
		}

		DriveCommand signal = new DriveCommand(lMotorSpeed, rMotorSpeed);

		return signal;
	}

	static double applyDeadband(double _in)
	{
		double sign = Math.signum(_in);							// get sign
		double mag = Math.abs(_in);								// get magnitude
		double out = 1.0/(1.0-kDeadband) * mag - kDeadband;		// inputs from kDeadband to 1.0 are scaled to outputs from 0 to 1
		out = sign * out;										// reapply the sign
		out = Util.limit(out, 1.0);								// limit to maximum of +/-1
		return out;
	}
}
