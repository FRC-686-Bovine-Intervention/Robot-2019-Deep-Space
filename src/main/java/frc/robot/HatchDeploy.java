package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Talon;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.Constants;

public class HatchDeploy {
    public static HatchDeploy mInstance = new HatchDeploy();

    public static HatchDeploy getInstance() {
        return mInstance;
    }

    public Talon dropMotor;
    public DoubleSolenoid topHatchSolenoid;
    public DoubleSolenoid bttmHatchSolenoid;
    private static DigitalInput limitSwitch;
    public final int dropPort = 4;
    public final int tFwdPort = 1;
    public final int tRvsPort = 2;
    public final int bFwdPort = 3;
    public final int bRvsPort = 4;
    public static double startingHeight = 1;
    public static double pickUpHeight = 0.75;
    public static double groundHeight = 0;
    //boolean limitSwitchTriggered = true; 

    INIT (startingHeight),
	TO_BUMPER (pickUpHeight),
    GROUND (groundHeight);
    // HatchDeployStateEnum(double startingHeight, double pickUpHeight, double groundHeight)  {
    //     this.startingHeight = startingHeight;
    //     this.pickUpHeight = pickUpHeight;
    //     this.groundHeight = groundHeight;
    // }
    // public HatchDeployStateEnum state = HatchDeployStateEnum.INIT;
    limitSwitch = new DigitalInput();

    public HatchDeploy() {
        topHatchSolenoid = new DoubleSolenoid(0, tFwdPort, tRvsPort);
        bttmHatchSolenoid = new DoubleSolenoid(0, bFwdPort, bRvsPort);
        dropMotor = new Talon(dropPort);
    }

    //    public void INIT ()
    //    {
    //     if (limitSwitchTriggered) 		{ set(HatchDeployStateEnum.TO_BUMPER); }
    //     if (Constants.kXboxButtonY) 		{ set(HatchDeployStateEnum.GROUND); }
    //    }
       
    //    case INIT
    //    talon.set(ControlMode.MotionMagic, inchesToEncoderUnits(target));
   
}

/*
JoystickControlsBase controls = ArcadeDriveJoystick.getInstance();
if (controls.getButton(Constants.kXboxButtonX)) {
    drop();
} else if (controls.getButton(Constants.kXboxButtonY)) {
    deploy();
} else {
    done();
}
public void drop() {
    dropMotor.set(floorLevel);
}

public void deploy() {
    topHatchSolenoid.set(DoubleSolenoid.Value.kForward);
    bttmHatchSolenoid.set(DoubleSolenoid.Value.kForward);
}

public void done() {
    topHatchSolenoid.set(DoubleSolenoid.Value.kReverse);
    bttmHatchSolenoid.set(DoubleSolenoid.Value.kReverse);
    dropMotor.set(highLevel);
}
*/