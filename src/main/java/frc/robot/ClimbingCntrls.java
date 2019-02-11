package  frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;

public class ClimbingCntrls {

    public static ClimbingCntrls mInstance = new ClimbingCntrls();
    public static ClimbingCntrls getInstance() { return mInstance; }
    public DoubleSolenoid climberSolenoid;
    public final int cFwdPort = 1;
    public final int cRvsPort = 2;

public ClimbingCntrls()
{
   climberSolenoid = new DoubleSolenoid(0, cFwdPort, cRvsPort);
}
    
public void run()
{
    JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
    if(controls.getButton(Constants.kXboxButtonLB))
    {
        //shooterMotor.set(shooterSpeed);
    }
    else 
    {
        //shooterMotor.set(shooterStop);
    }
}

}