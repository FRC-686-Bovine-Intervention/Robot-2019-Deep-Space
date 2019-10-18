package frc.robot.lib.joystick;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.SmartDashboardInteractions;
import frc.robot.command_status.DriveCommand;

/**
 * Stores what Joystick configuration is selected in SmartDashboard.
 * To be updated at startup and TeleopInit()
 */
public class SelectedJoystick
{
	private static SelectedJoystick instance = null;

	public static SelectedJoystick getInstance() {
		if (instance == null) {
			instance = new SelectedJoystick();
		}
		return instance;
    }    

    JoystickControlsBase controls = TmReversibleArcadeDriveJoystick.getInstance();

    public SelectedJoystick()
    {
        update();
    }

    public void update()
    {
        controls = SmartDashboardInteractions.getInstance().getJoystickControlsMode();
    }

    public JoystickControlsBase get()
    {
        return controls;
    }

    // pass-thru gets
    public DriveCommand getDriveCommand()    { return controls.getDriveCommand(); }    
    public boolean getButton(int _joystickNum, int _buttonNum) { return controls.getButton(_joystickNum, _buttonNum); }
    public int getPOV(int _joystickNum) { return controls.getPOV(_joystickNum); }    
    public boolean getAxisAsButton(int _joystickNum, int _buttonNum) { return controls.getAxisAsButton(_joystickNum, _buttonNum); }
    public boolean getDrivingCargo()       { return controls.usingLeftStick(); }
    public void setRumble(GenericHID.RumbleType _rumbleType, double _value)       { controls.setRumble(_rumbleType, _value); }
    public boolean joystickActive() { return controls.joystickActive(); }

    // default to joystick 0 if not provided
    public boolean getButton( int _buttonNum) { return controls.getButton(0, _buttonNum); }
    public int getPOV() { return controls.getPOV(0); }    
    public boolean getAxisAsButton(int _buttonNum) { return controls.getAxisAsButton(0, _buttonNum); }
}