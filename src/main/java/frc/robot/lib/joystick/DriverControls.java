package frc.robot.lib.joystick;

import frc.robot.SmartDashboardInteractions;



public class DriverControls
{
    private static DriverControls mInstance = new DriverControls();

    public static DriverControls getInstance() 
    {
        return mInstance;
    }

    public enum SchemeEnum 
    {
        XBOX("XBox"),
        THRUSTMASTER("ThrustMaster"),
        MICHAEL("Michael"),
        TYLER("Tyler"),				
        BEN("Ben");
 
    	public final String name;
    	
        SchemeEnum(String name) {
    		this.name= name;
    	}
    }

    // button board constants
    public static int kCargoIntakeRocketButton =    ButtonBoard.kButtonBoardB;
    public static int kCargoIntakeCargoShipButton = ButtonBoard.kButtonBoardA;
    public static int kDefenseButton =              ButtonBoard.kButtonBoardRB;
    public static int kClimbingStartButton =        ButtonBoard.kButtonBoardLB;
    public static int kClimbingExtendButton =       ButtonBoard.kButtonBoardX;
    public static int kClimbingRetractButton =      ButtonBoard.kButtonBoardY;
    public static int kEmergencyZeroingAxis =       ButtonBoard.kButtonBoardSR;

    public SchemeEnum controlScheme;

    public DriverControls() 
    {
    }

    public void updateDriverControlScheme() 
    {
        controlScheme = SmartDashboardInteractions.getInstance().getDriverControlsScheme();
    }

    public boolean getBoolean( DriverControlsEnum _control ) 
    {
        SelectedJoystick controls = SelectedJoystick.getInstance();
        ButtonBoard buttonBoard = ButtonBoard.getInstance();

        switch (controlScheme)  // control scheme selected in SmartDashboard
        {
            case XBOX:
            default:
                switch (_control)
                {
                    case VISION_ASSIST:                 return controls.getButton(XboxConstants.kXboxButtonA);
                    case HATCH_DEPLOY:                  return controls.getButton(XboxConstants.kXboxButtonRB);
                    case HATCH_SHOOT:                   return controls.getButton(XboxConstants.kXboxRTriggerAxis);
                    case CARGO_INTAKE:                  return controls.getButton(XboxConstants.kXboxButtonLB);
                    case CARGO_OUTTAKE:                 return controls.getButton(XboxConstants.kXboxLTriggerAxis);
                    case CARGO_INTAKE_DEPOT_HEIGHT:     return controls.getButton(XboxConstants.kXboxButtonX);
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
                // end case XBOX
    
                case THRUSTMASTER:
                switch (_control)
                {
                    case VISION_ASSIST:                 return controls.getButton(ThrustMasterConstants.kLeftStick,  ThrustMasterConstants.kLeftThumbButton);
                    case HATCH_DEPLOY:                  return controls.getButton(ThrustMasterConstants.kRightStick, ThrustMasterConstants.kBottomThumbButton);
                    case HATCH_SHOOT:                   return controls.getButton(ThrustMasterConstants.kRightStick, ThrustMasterConstants.kTriggerButton);
                    case CARGO_INTAKE:                  return controls.getButton(ThrustMasterConstants.kLeftStick,  ThrustMasterConstants.kBottomThumbButton);
                    case CARGO_OUTTAKE:                 return controls.getButton(ThrustMasterConstants.kLeftStick,  ThrustMasterConstants.kTriggerButton);
                    case CARGO_INTAKE_DEPOT_HEIGHT:     return controls.getButton(ThrustMasterConstants.kLeftStick,  ThrustMasterConstants.kRightThumbButton);
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
                // end case THRUSTMASTER

                case MICHAEL:   return false;
                    // TODO: replace above 'return false' with modified copy of above
                // end case MICHAEL

                case TYLER:   return false;
                    // TODO: replace above 'return false' with modified copy of above
                // end case TYLER

                case BEN:   return false;
                    // TODO: replace above 'return false' with modified copy of above
                // end case BEN
            }
    }

}
