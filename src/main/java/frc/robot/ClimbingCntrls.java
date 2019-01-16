package  frc.robot;

import edu.wpi.first.wpilibj.Talon;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;

public class ClimbingCntrls {

    public static ClimbingCntrls mInstance = new ClimbingCntrls();
    public static ClimbingCntrls getInstance() { return mInstance; }
    public Talon shooterMotor;
    public static int shooterPort = 2;
    public static double shooterSpeed = 1;
    public static double shooterStop = 0;

public ClimbingCntrls()
{
 shooterMotor = new Talon(shooterPort);
}
    
public void run()
{
    JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
    if(controls.getButton(Constants.kXboxButtonLB))
    {
        shooterMotor.set(shooterSpeed);
    }
    else 
    {
        shooterMotor.set(shooterStop);
    }
}

}