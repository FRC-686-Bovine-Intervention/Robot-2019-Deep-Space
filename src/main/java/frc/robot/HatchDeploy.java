package  frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.Constants;

public class HatchDeploy {
    public static HatchDeploy mInstance = new HatchDeploy();
    public static HatchDeploy getInstance() { return mInstance; }
    public Talon dropMotor; 
    public DoubleSolenoid topHatchSolenoid;
    public DoubleSolenoid bttmHatchSolenoid;
    public final int dropPort = 4;
    public final int tFwdPort = 5;
    public final int tRvsPort = 6;
    public final int bFwdPort = 7;
    public final int bRvsPort = 8;
    public static double floorLevel = 0;

    public HatchDeploy()
    {
     topHatchSolenoid = new DoubleSolenoid(0, tFwdPort, tRvsPort);
     bttmHatchSolenoid = new DoubleSolenoid(0, bFwdPort, bRvsPort);
     dropMotor = new Talon(dropPort);
    }


    public void run()
    {
        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
        if (controls.getButton(Constants.kXboxButtonX))
        {
            dropMotor.set(floorLevel);
        }
        else if (controls.getButton(Constants.kXboxButtonY)) {
            topHatchSolenoid.set(DoubleSolenoid.Value.kForward);
            bttmHatchSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
          topHatchSolenoid.set(DoubleSolenoid.Value.kReverse);
          bttmHatchSolenoid.set(DoubleSolenoid.Value.kReverse);
         }
    }
}
