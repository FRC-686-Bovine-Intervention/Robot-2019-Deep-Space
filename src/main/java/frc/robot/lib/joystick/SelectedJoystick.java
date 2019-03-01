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

    JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();

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
    public boolean getButton(int _num)       { return controls.getButton(_num); }
    public boolean getAxisAsButton(int _num) { return controls.getAxisAsButton(_num); }
    public boolean getDrivingCargo()       { return controls.usingLeftStick(); }
    public void setRumble(GenericHID.RumbleType _rumbleType, double _value)       { controls.setRumble(_rumbleType, _value); }
    public boolean joystickActive() { return controls.joystickActive(); }
}