package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.util.DataLogger;



public abstract class DriverControlsBase
{
    // control to get left and right motor speeds
    public abstract DriveCommand getDriveCommand();

    // control to get button presses, etc.
    public abstract boolean getBoolean( DriverControlsEnum _control );

    // logging
    public abstract DataLogger getLogger();

}
