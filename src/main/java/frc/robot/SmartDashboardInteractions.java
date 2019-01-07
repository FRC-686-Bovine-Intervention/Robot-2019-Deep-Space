package frc.robot;

import frc.robot.auto.AutoModeBase;
import frc.robot.auto.modes.*;

import frc.robot.lib.joystick.*;

import frc.robot.lib.util.Pose;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
public class SmartDashboardInteractions 
{
    static SendableChooser<StartPositionOption> startChooser;

    public enum StartPositionOption
    {
        LEFT_START("Left", FieldDimensions.getLeftStartPose()),
        CENTER_START("Center", FieldDimensions.getCenterStartPose()),
        RIGHT_START("Right", FieldDimensions.getRightStartPose());

        public final String name;
        public final Pose initialPose;

        StartPositionOption(String name, Pose initialPose) {
            this.name = name;
            this.initialPose = initialPose;
        }
    }

    
    
    SendableChooser<StartDelayOption> startDelayChooser;
    
    public enum StartDelayOption
    {
    	DELAY_0_SEC("0 Sec", 0.0),
    	DELAY_1_SEC("1 Sec", 1.0),
    	DELAY_2_SEC("2 Sec", 2.0),
    	DELAY_3_SEC("3 Sec", 3.0),
    	DELAY_4_SEC("4 Sec", 4.0),
    	DELAY_5_SEC("5 Sec", 5.0);
    
    	public final String name;
    	public final double delaySec;
    	
    	StartDelayOption(String name, double delaySec) {
    		this.name= name;
    		this.delaySec = delaySec;
    	}
    }
   
    
    static SendableChooser<AutoModeOption> autoModeChooser;
    
    enum AutoModeOption
    {
        STAND_STILL("Stand Still");
    	
        public final String name;

        AutoModeOption(String name) {
            this.name = name;
        }
    }


    SendableChooser<JoystickOption> joystickModeChooser;
    
    enum JoystickOption 
    {
        ARCADE_DRIVE("Arcade Drive"),
        TRIGGER_DRIVE("Trigger Drive"),				// works for Xbox controller and Xbox steering wheel
        TANK_DRIVE("Tank Drive"),
        CHEESY_ARCADE_DRIVE("Cheesy Arcade Drive"),
        CHEESY_TRIGGER_DRIVE("Cheesy Trigger Drive"),
        CHEESY_2STICK_DRIVE("Cheesy Two-Stick Drive");
        public final String name;

        JoystickOption(String name) {
            this.name = name;
        }
    }
    
    
    public void initWithDefaults() 
    {
        startChooser = new SendableChooser<StartPositionOption>();
        startChooser.addDefault(StartPositionOption.LEFT_START.toString(),    StartPositionOption.LEFT_START);
        startChooser.addObject(StartPositionOption.CENTER_START.toString(),    StartPositionOption.CENTER_START);
        startChooser.addObject(StartPositionOption.RIGHT_START.toString(),    StartPositionOption.RIGHT_START);
        SmartDashboard.putData("Start Position", startChooser);
        
        startDelayChooser = new SendableChooser<StartDelayOption>();
        startDelayChooser.addDefault(StartDelayOption.DELAY_0_SEC.toString(), StartDelayOption.DELAY_0_SEC);
        startDelayChooser.addObject(StartDelayOption.DELAY_1_SEC.toString(), StartDelayOption.DELAY_1_SEC);
        startDelayChooser.addObject(StartDelayOption.DELAY_2_SEC.toString(), StartDelayOption.DELAY_2_SEC);
        startDelayChooser.addObject(StartDelayOption.DELAY_3_SEC.toString(), StartDelayOption.DELAY_3_SEC);
        startDelayChooser.addObject(StartDelayOption.DELAY_4_SEC.toString(), StartDelayOption.DELAY_4_SEC);
        startDelayChooser.addObject(StartDelayOption.DELAY_5_SEC.toString(), StartDelayOption.DELAY_5_SEC);
        SmartDashboard.putData("Auto Start Delay", startDelayChooser);
       
        autoModeChooser = new SendableChooser<AutoModeOption>();
        autoModeChooser.addObject(AutoModeOption.STAND_STILL.name, AutoModeOption.STAND_STILL);
        SmartDashboard.putData("Auto Mode", autoModeChooser);
    	
    	joystickModeChooser = new SendableChooser<JoystickOption>();
    	joystickModeChooser.addObject(JoystickOption.ARCADE_DRIVE.name,        JoystickOption.ARCADE_DRIVE);
    	joystickModeChooser.addDefault(JoystickOption.TRIGGER_DRIVE.name,        JoystickOption.TRIGGER_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.TANK_DRIVE.name, 	      JoystickOption.TANK_DRIVE);
     	joystickModeChooser.addObject(JoystickOption.CHEESY_ARCADE_DRIVE.name,  JoystickOption.CHEESY_ARCADE_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.CHEESY_TRIGGER_DRIVE.name, JoystickOption.CHEESY_TRIGGER_DRIVE);
    	joystickModeChooser.addObject(JoystickOption.CHEESY_2STICK_DRIVE.name,  JoystickOption.CHEESY_2STICK_DRIVE);
    	SmartDashboard.putData("Joystick Chooser", joystickModeChooser);
    	
     }
    
    
    
    public AutoModeBase getAutoModeSelection()
    {
    	AutoModeOption autoMode = (AutoModeOption)autoModeChooser.getSelected();

    	String gameData = DriverStation.getInstance().getGameSpecificMessage();
    	while(gameData.length() < 1){
    		gameData = DriverStation.getInstance().getGameSpecificMessage(); 
    	}
    	
    	switch(autoMode)
    	{
    	case STAND_STILL:
			return new StandStillMode();
			
    	default:
            System.out.println("ERROR: unexpected auto mode: " + autoMode);
			return new StandStillMode();
    	}
    }

   

    public JoystickControlsBase getJoystickControlsMode() 
    {
    	JoystickOption selMode = (JoystickOption)joystickModeChooser.getSelected(); 
    	
   	
    	
    	switch (selMode)
    	{
    	case ARCADE_DRIVE:
			return ArcadeDriveJoystick.getInstance();
			
    	case TRIGGER_DRIVE:
			return TriggerDriveJoystick.getInstance();

    	case TANK_DRIVE:
    		return TankDriveJoystick.getInstance();

    	case CHEESY_ARCADE_DRIVE:
    		return CheesyArcadeDriveJoystick.getInstance();

    	case CHEESY_TRIGGER_DRIVE:
    		return CheesyTriggerDriveJoystick.getInstance();

    	case CHEESY_2STICK_DRIVE:
    		return CheesyTwoStickDriveJoystick.getInstance();


    	default:
            System.out.println("ERROR: unexpected joystick selection: " + selMode);
			return ArcadeDriveJoystick.getInstance();
    	}
    
    }
}
    
   


