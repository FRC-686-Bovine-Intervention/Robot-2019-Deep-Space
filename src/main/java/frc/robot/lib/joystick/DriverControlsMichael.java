package frc.robot.lib.joystick;

import frc.robot.CargoIntake;
import frc.robot.CargoIntake.CargoDeployPositionEnum;
import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.joystick.SteeringLib.NonLinearityEnum;
import frc.robot.lib.util.DataLogger;
import edu.wpi.first.wpilibj.Timer;


public class DriverControlsMichael extends ReversibleDriverControlsBase
{
	// singleton class
    private static DriverControlsMichael instance = null;
    public static DriverControlsMichael getInstance() 
    { 
        if (instance == null) {
            instance = new DriverControlsMichael();
        }
        return instance;
    }
    
    
    // Joystick Port Constants
    public static int kLeftStickPort = 0;
    public static int kRightStickPort = 1;
    public static int kButtonBoardPort = 2;

    public static JoystickBase lStick;
    public static JoystickBase rStick;
    public static JoystickBase buttonBoard;

    public double prevMatchTime = 0;
    public int prevPOV = -1;
    boolean povPressed = false;
    boolean povDown = false;

    public static ReversibleSteeringBase steeringControls;

    // button board constants
    public static int kCargoIntakeRocketButton =    ButtonBoard.kButtonB;
    public static int kCargoIntakeCargoShipButton = ButtonBoard.kButtonA;
    public static int kDefenseButton =              ButtonBoard.kButtonRB;
    public static int kClimbingStartButton =        ButtonBoard.kButtonLB;
    public static int kClimbingExtendButton =       ButtonBoard.kButtonX;
    public static int kClimbingRetractButton =      ButtonBoard.kButtonY;
    public static int kEmergencyZeroingAxis =       ButtonBoard.kButtonSR;

    public DriverControlsMichael() 
    {
        lStick = new Thrustmaster(kLeftStickPort);
        rStick = new Thrustmaster(kRightStickPort);
        buttonBoard = new ButtonBoard(kButtonBoardPort);

        double throttleDeadband =     0.02;
        double turnDeadband =         0.02;
        NonLinearityEnum throttleNonLinearity = NonLinearityEnum.SQUARED;
        NonLinearityEnum turnNonLinearity =     NonLinearityEnum.SQUARED;

        steeringControls = new TmReversibleArcadeDriveSteering(lStick, rStick, new DeadbandNonLinearity(throttleDeadband, turnDeadband, throttleNonLinearity, turnNonLinearity));
    }

    public DriveCommand getDriveCommand() 
    {
        return steeringControls.getDriveCommand(); 
    }


    public boolean getBoolean( DriverControlsEnum _control ) 
    {
        if (CheckForNewDS.getInstance().check())
        {
            // update these only once per DS packet
            prevPOV = lStick.getPOV();
            povPressed =  (prevPOV == -1) && (lStick.getPOV() != prevPOV);
            povDown = lStick.getPOV() >= 135 && lStick.getPOV() <= 225;
        }
 
        switch (_control)
        {
            case VISION_ASSIST:                 return lStick.getButton(Thrustmaster.kLeftThumbButton);
            case HATCH_DEPLOY:                  return rStick.getPOV() == 45 || rStick.getPOV() == 315 || rStick.getPOV() == 0;
            case HATCH_SHOOT:                   return rStick.getPOV() <= 225 && rStick.getPOV() >= 135;
            case CARGO_INTAKE:                  return povPressed && povDown && CargoIntake.getInstance().getTarget() != CargoDeployPositionEnum.GROUND;
            case CARGO_OUTTAKE:                 return lStick.getPOV() == 0;
            case CARGO_INTAKE_DEPOT_HEIGHT:     return povPressed && povDown && CargoIntake.getInstance().getTarget() == CargoDeployPositionEnum.GROUND;
            case CARGO_INTAKE_ROCKET_HEIGHT:    return lStick.getButton(Thrustmaster.kRightThumbButton);
            case CARGO_INTAKE_CARGO_HEIGHT:     return lStick.getButton(Thrustmaster.kBottomThumbButton);
            case DEFENSE:                       return rStick.getButton(Thrustmaster.kTriggerButton);
            case CLIMB_PREPARE:                 return rStick.getButton(Thrustmaster.kTopButton4);
            case CLIMB_EXTEND:                  return rStick.getButton(Thrustmaster.kTopButton5);
            case CLIMB_RETRACT:                 return rStick.getButton(Thrustmaster.kTopButton6);
            case EMERGENCY_ZEROING:             return buttonBoard.getButton(kEmergencyZeroingAxis);
            case QUICK_TURN:                    return false;
            default:                            return false;
        }
    }

    public boolean getDrivingCargo()
    {
        return steeringControls.usingLeftStick();
    }

    public boolean joystickActive()
    {
        return steeringControls.joystickActive();
    }



    public DataLogger getLogger() { return logger; }
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {
            if (lStick != null)             { lStick.getLogger().log(); }
            if (rStick != null)             { rStick.getLogger().log(); }
            if (buttonBoard != null)        { buttonBoard.getLogger().log(); }
            if (steeringControls != null)   { steeringControls.getLogger().log(); }
        }
    };    

}
