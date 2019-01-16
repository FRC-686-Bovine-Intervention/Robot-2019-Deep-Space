package  frc.robot;

import edu.wpi.first.wpilibj.Talon;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;

public class CargoBallIntake {

    public static CargoBallIntake mInstance = new CargoBallIntake();
    public static CargoBallIntake getInstance() { return mInstance; }
    public Talon cargoBallMotor;
    public Talon liftMechanism;
    public static int liftPort = 1;
    public static int cargoPort = 2;
    public static double rocketHeight = 0.65;
    public static double groundHeight = 0.25;
    public static double cargoBallSpeed = 1;
    public static double cargoBallStop = 0;

public CargoBallIntake()
{
 cargoBallMotor = new Talon(cargoPort);
 liftMechanism = new Talon(liftPort);
}
    
public void run()
{
    JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
    if(controls.getButton(Constants.kXboxButtonLB))
    {
        cargoBallMotor.set(cargoBallSpeed);
        liftMechanism.setPosition(groundHeight);
    }
    else // if(proximitySensor)
    {
        cargoBallMotor.set(cargoBallStop);
        liftMechanism.setPosition(rocketHeight);
    }
}

}