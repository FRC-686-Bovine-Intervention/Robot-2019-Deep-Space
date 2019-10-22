package frc.robot.lib.joystick;

import frc.robot.command_status.DriveCommand;
import frc.robot.lib.joystick.SteeringLib.DeadbandNonLinearity;
import frc.robot.lib.joystick.SteeringLib.NonLinearityEnum;
import frc.robot.lib.util.DataLogger;



public class DriverControlsReversibleXbox extends ReversibleDriverControlsBase
{
	// singleton class
    private static DriverControlsReversibleXbox instance = null;
    public static DriverControlsReversibleXbox getInstance() 
    { 
        if (instance == null) {
            instance = new DriverControlsReversibleXbox();
        }
        return instance;
    }



    // Joystick Port Constants
    public static int kXboxStickPort   = 0;
    public static int kButtonBoardPort = 1;

    public static JoystickBase stick;
    public static JoystickBase buttonBoard;

    public static ReversibleSteeringBase steeringControls;

    // button board constants
    public static int kCargoIntakeRocketButton =    ButtonBoard.kButtonB;
    public static int kCargoIntakeCargoShipButton = ButtonBoard.kButtonA;
    public static int kDefenseButton =              ButtonBoard.kButtonRB;
    public static int kClimbingStartButton =        ButtonBoard.kButtonLB;
    public static int kClimbingExtendButton =       ButtonBoard.kButtonX;
    public static int kClimbingRetractButton =      ButtonBoard.kButtonY;
    public static int kEmergencyZeroingAxis =       ButtonBoard.kButtonSR;

    public DriverControlsReversibleXbox() 
    {
        stick = new Thrustmaster(kXboxStickPort);
        buttonBoard = new ButtonBoard(kButtonBoardPort);

        double throttleDeadband =     0.02;
        double turnDeadband =         0.02;
        NonLinearityEnum throttleNonLinearity = NonLinearityEnum.SQUARED;
        NonLinearityEnum turnNonLinearity =     NonLinearityEnum.SQUARED;

        steeringControls = new XboxReversibleArcadeDriveSteering(stick, new DeadbandNonLinearity(throttleDeadband, turnDeadband, throttleNonLinearity, turnNonLinearity));
    }

    public DriveCommand getDriveCommand() 
    {
        return steeringControls.getDriveCommand(); 
    }


    public boolean getBoolean( DriverControlsEnum _control ) 
    {
        switch (_control)
        {
            case VISION_ASSIST:                 return stick.getButton(Xbox.kButtonA);
            case HATCH_DEPLOY:                  return stick.getButton(Xbox.kButtonRB);
            case HATCH_SHOOT:                   return stick.getButton(Xbox.kRTriggerAxis);
            case CARGO_INTAKE:                  return stick.getButton(Xbox.kButtonLB);
            case CARGO_OUTTAKE:                 return stick.getButton(Xbox.kLTriggerAxis);
            case CARGO_INTAKE_DEPOT_HEIGHT:     return stick.getButton(Xbox.kButtonX);
            case CARGO_INTAKE_ROCKET_HEIGHT:    return buttonBoard.getButton(kCargoIntakeRocketButton);
            case CARGO_INTAKE_CARGO_HEIGHT:     return buttonBoard.getButton(kCargoIntakeCargoShipButton);
            case DEFENSE:                       return buttonBoard.getButton(kDefenseButton);
            case CLIMB_PREPARE:                 return buttonBoard.getButton(kClimbingStartButton);
            case CLIMB_EXTEND:                  return buttonBoard.getButton(kClimbingExtendButton);
            case CLIMB_RETRACT:                 return buttonBoard.getButton(kClimbingRetractButton);
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
            if (stick != null)              { stick.getLogger().log(); }
            if (buttonBoard != null)        { buttonBoard.getLogger().log(); }
            if (steeringControls != null)   { steeringControls.getLogger().log(); }
        }
    };        
}
