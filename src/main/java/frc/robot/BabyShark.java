package  frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.Constants;

public class BabyShark {
    public static BabyShark mInstance = new BabyShark();
    public static BabyShark getInstance() { return mInstance; }
    public DoubleSolenoid hatchSolenoid;
    public final int hFwdPort = 5;
    public final int hRvsPort = 6;

    public BabyShark()
    {
     hatchSolenoid = new DoubleSolenoid(0, hFwdPort, hRvsPort);
    }


    public void run()
    {
        JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
        if (controls.getButton(Constants.kXboxButtonY)) {
            hatchSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
          hatchSolenoid.set(DoubleSolenoid.Value.kReverse);
         }
    }
}
