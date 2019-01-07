package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;

import edu.wpi.first.wpilibj.Joystick;

/**
 * An abstract class that forms the base of joystick controls.
 */
public abstract class JoystickControlsBase 
{
    protected final Joystick mStick;

    public static double kJoystickDeadzone = 0.2;   // deadzone at center of joystick extends from +/-kJoystickDeadzone

    protected JoystickControlsBase() 
    {
        mStick = new Joystick(0);
    }

    // DRIVER CONTROLS
    public abstract DriveCommand getDriveCommand();	// mapping from joystick controls to DriveSignal
    
    public boolean getButton(int _num) { return mStick.getRawButton(_num); }
    
}
