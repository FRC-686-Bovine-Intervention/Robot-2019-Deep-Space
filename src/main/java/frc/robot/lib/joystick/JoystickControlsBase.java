package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.command_status.DriveCommand;

/**
 * An abstract class that forms the base of joystick controls.
 */
public abstract class JoystickControlsBase 
{
    protected final Joystick mStick;
    protected boolean drivingForward = true;
    public static double kJoystickDeadzone = 0.2;   // deadzone at center of joystick extends from +/-kJoystickDeadzone

    protected JoystickControlsBase() 
    {
        mStick = new Joystick(0);
    }

    // DRIVER CONTROLS
    public abstract DriveCommand getDriveCommand();	// mapping from joystick controls to DriveSignal
    
    public boolean getButton(int _num) { return mStick.getRawButton(_num); }
    public boolean getAxisAsButton(int _num) { return (mStick.getRawAxis(_num) > 0.5); }

    public boolean usingLeftStick() { return drivingForward; }
    public boolean joystickActive() { return true; }

    public void setRumble(GenericHID.RumbleType _rumbleType, double _value)       { mStick.setRumble(_rumbleType, _value); }    
    
}
