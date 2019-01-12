package  frc.robot;

import edu.wpi.first.wpilibj.Talon;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;

public class CargoBallIntake {

    public static CargoBallIntake mInstance = new CargoBallIntake();
    public static CargoBallIntake getInstance() { return mInstance; }
    public Talon cargoBallMotor;
    public static int port = 2;
    public static double cargoBallSpeed = 1;
    public static double cargoBallStop = 0;

public CargoBallIntake()
{
 cargoBallMotor = new Talon(port);
}
    
public void run()
{
    JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
    if(controls.getButton(Constants.kXboxButtonLB))
    {
        cargoBallMotor.set(cargoBallSpeed);
    }
    else if(controls.getButton(Constants.kXboxButtonRB))
    {
        cargoBallMotor.set(cargoBallStop);
    }
}

}