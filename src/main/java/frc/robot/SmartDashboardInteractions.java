package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.modes.DebugAuto;
import frc.robot.auto.modes.FieldDimensions;
import frc.robot.auto.modes.HatchAutoBethesda;
import frc.robot.auto.modes.HatchAutoChamps;
import frc.robot.auto.modes.StandStillMode;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.lib.joystick.ReversibleArcadeDriveJoystick;
import frc.robot.lib.util.Pose;

/**
 * Controls the interactive elements of SmartDashboard.
 *
 * Keeps the network tables keys in one spot and enforces autonomous mode
 * invariants.
 */
public class SmartDashboardInteractions 
{
	private static SmartDashboardInteractions instance = null;

	public static SmartDashboardInteractions getInstance() {
		if (instance == null) {
			instance = new SmartDashboardInteractions();
		}
		return instance;
    }
        
    public SmartDashboardInteractions()
    {
        initWithDefaults();
    }


    public void initWithDefaults() 
    {
    	joystickModeChooser = new SendableChooser<JoystickOption>();
    	// joystickModeChooser.addOption(JoystickOption.ARCADE_DRIVE.name,        JoystickOption.ARCADE_DRIVE);
    	joystickModeChooser.setDefaultOption(JoystickOption.REVERSIBLE_ARCADE_DRIVE.toString(),        JoystickOption.REVERSIBLE_ARCADE_DRIVE);
		// joystickModeChooser.addOption(JoystickOption.TRIGGER_DRIVE.name,        JoystickOption.TRIGGER_DRIVE);
    	// joystickModeChooser.addOption(JoystickOption.TANK_DRIVE.name, 	      JoystickOption.TANK_DRIVE);
     	// joystickModeChooser.addOption(JoystickOption.CHEESY_ARCADE_DRIVE.name,  JoystickOption.CHEESY_ARCADE_DRIVE);
    	// joystickModeChooser.addOption(JoystickOption.CHEESY_TRIGGER_DRIVE.name, JoystickOption.CHEESY_TRIGGER_DRIVE);
    	// joystickModeChooser.addOption(JoystickOption.CHEESY_2STICK_DRIVE.name,  JoystickOption.CHEESY_2STICK_DRIVE);
    	SmartDashboard.putData("Joystick Chooser", joystickModeChooser);

        autoModeChooser = new SendableChooser<AutoModeOption>();
        autoModeChooser.setDefaultOption(AutoModeOption.HATCH_AUTO.name, AutoModeOption.HATCH_AUTO);
        autoModeChooser.addOption(AutoModeOption.STAND_STILL.name, AutoModeOption.STAND_STILL);
        autoModeChooser.addOption(AutoModeOption.DEBUG_AUTO.name, AutoModeOption.DEBUG_AUTO);
        // autoModeChooser.setDefaultOption(AutoModeOption.DEBUG_AUTO.name, AutoModeOption.DEBUG_AUTO);
        SmartDashboard.putData("Auto Mode", autoModeChooser);
    	
        startPositionChooser = new SendableChooser<StartPositionOption>();
        startPositionChooser.addOption(StartPositionOption.LEFT_START.toString(),    StartPositionOption.LEFT_START);
        startPositionChooser.addOption(StartPositionOption.HAB2_LEFT_START.toString(),    StartPositionOption.HAB2_LEFT_START);
        startPositionChooser.addOption(StartPositionOption.CENTER_LEFT_START.toString(),    StartPositionOption.CENTER_LEFT_START);
        startPositionChooser.setDefaultOption(StartPositionOption.CENTER_RIGHT_START.toString(),    StartPositionOption.CENTER_RIGHT_START);
        startPositionChooser.addOption(StartPositionOption.RIGHT_START.toString(),    StartPositionOption.RIGHT_START);
        startPositionChooser.addOption(StartPositionOption.HAB2_RIGHT_START.toString(),    StartPositionOption.HAB2_RIGHT_START);
        SmartDashboard.putData("Start Position", startPositionChooser);

        startDelayChooser = new SendableChooser<StartDelayOption>();
        startDelayChooser.setDefaultOption(StartDelayOption.DELAY_0_SEC.toString(), StartDelayOption.DELAY_0_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_1_SEC.toString(), StartDelayOption.DELAY_1_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_2_SEC.toString(), StartDelayOption.DELAY_2_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_3_SEC.toString(), StartDelayOption.DELAY_3_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_4_SEC.toString(), StartDelayOption.DELAY_4_SEC);
        startDelayChooser.addOption(StartDelayOption.DELAY_5_SEC.toString(), StartDelayOption.DELAY_5_SEC);
        SmartDashboard.putData("Auto Start Delay", startDelayChooser);

        firstTargetChooser = new SendableChooser<FieldDimensions.TargetPositionEnum>();
        firstTargetChooser.setDefaultOption(FieldDimensions.TargetPositionEnum.CARGO_FRONT.toString(), FieldDimensions.TargetPositionEnum.CARGO_FRONT);
        firstTargetChooser.addOption(FieldDimensions.TargetPositionEnum.CARGO_SIDE1.toString(), FieldDimensions.TargetPositionEnum.CARGO_SIDE1);
        firstTargetChooser.addOption(FieldDimensions.TargetPositionEnum.CARGO_SIDE2.toString(), FieldDimensions.TargetPositionEnum.CARGO_SIDE2);
        firstTargetChooser.addOption(FieldDimensions.TargetPositionEnum.CARGO_SIDE3.toString(), FieldDimensions.TargetPositionEnum.CARGO_SIDE3);
        firstTargetChooser.addOption(FieldDimensions.TargetPositionEnum.ROCKET_NEAR.toString(), FieldDimensions.TargetPositionEnum.ROCKET_NEAR);
        firstTargetChooser.addOption(FieldDimensions.TargetPositionEnum.ROCKET_FAR.toString(),  FieldDimensions.TargetPositionEnum.ROCKET_FAR);
        SmartDashboard.putData("First Auto Target", firstTargetChooser);

        secondTargetChooser = new SendableChooser<FieldDimensions.TargetPositionEnum>();
        secondTargetChooser.setDefaultOption(FieldDimensions.TargetPositionEnum.CARGO_FRONT.toString(), FieldDimensions.TargetPositionEnum.CARGO_FRONT);
        secondTargetChooser.addOption(FieldDimensions.TargetPositionEnum.CARGO_SIDE1.toString(), FieldDimensions.TargetPositionEnum.CARGO_SIDE1);
        secondTargetChooser.addOption(FieldDimensions.TargetPositionEnum.CARGO_SIDE2.toString(), FieldDimensions.TargetPositionEnum.CARGO_SIDE2);
        secondTargetChooser.addOption(FieldDimensions.TargetPositionEnum.CARGO_SIDE3.toString(), FieldDimensions.TargetPositionEnum.CARGO_SIDE3);
        secondTargetChooser.addOption(FieldDimensions.TargetPositionEnum.ROCKET_NEAR.toString(), FieldDimensions.TargetPositionEnum.ROCKET_NEAR);
        secondTargetChooser.addOption(FieldDimensions.TargetPositionEnum.ROCKET_FAR.toString(),  FieldDimensions.TargetPositionEnum.ROCKET_FAR);
        SmartDashboard.putData("Second Auto Target", secondTargetChooser);
    }

        
    
    
    SendableChooser<JoystickOption> joystickModeChooser;
    
    enum JoystickOption 
    {
        // ARCADE_DRIVE("Arcade Drive"),
        REVERSIBLE_ARCADE_DRIVE("Reversible Arcade Drive");
        // TRIGGER_DRIVE("Trigger Drive"),				// works for Xbox controller and Xbox steering wheel
        // TANK_DRIVE("Tank Drive"),
        // CHEESY_ARCADE_DRIVE("Cheesy Arcade Drive"),
        // CHEESY_TRIGGER_DRIVE("Cheesy Trigger Drive"),
        // CHEESY_2STICK_DRIVE("Cheesy Two-Stick Drive");
    	public final String name;
    	
        JoystickOption(String name) {
    		this.name= name;
    	}
    }
   
    public JoystickControlsBase getJoystickControlsMode() 
    {
    	JoystickOption selMode = (JoystickOption)joystickModeChooser.getSelected(); 
    
    	switch (selMode)
    	{
    	// case ARCADE_DRIVE:
		// 	return ArcadeDriveJoystick.getInstance();
			
		case REVERSIBLE_ARCADE_DRIVE:
			return ReversibleArcadeDriveJoystick.getInstance();
    
    	// case TRIGGER_DRIVE:
		// 	return TriggerDriveJoystick.getInstance();
    	
    	// case TANK_DRIVE:
    	// 	return TankDriveJoystick.getInstance();

    	// case CHEESY_ARCADE_DRIVE:
    	// 	return CheesyArcadeDriveJoystick.getInstance();

    	// case CHEESY_TRIGGER_DRIVE:
    	// 	return CheesyTriggerDriveJoystick.getInstance();

    	// case CHEESY_2STICK_DRIVE:
    	// 	return CheesyTwoStickDriveJoystick.getInstance();
    

    	default:
            System.out.println("ERROR: unexpected joystick selection: " + selMode);
			return ReversibleArcadeDriveJoystick.getInstance();
        }
    
    }
    
    
        
       
    	
    static SendableChooser<AutoModeOption> autoModeChooser;
    
    enum AutoModeOption
    {
        HATCH_AUTO("Hatch Panel Auto"),
        STAND_STILL("Stand Still"),
        DEBUG_AUTO("Debug");
    
        public final String name;
    
        AutoModeOption(String name) {
            this.name = name;
        }
    }
    
    public AutoModeBase getAutoModeSelection()
    {
    	AutoModeOption autoMode = (AutoModeOption)autoModeChooser.getSelected();

    	switch(autoMode)
    	{
        case HATCH_AUTO:
            return new HatchAutoChamps(); //changed from HatchAuto 
			
    	case STAND_STILL:
            return new StandStillMode();

        case DEBUG_AUTO:
            return new DebugAuto();
			
    	default:
            System.out.println("ERROR: unexpected auto mode: " + autoMode);
			return new StandStillMode();
    	}
    }

   



    static SendableChooser<StartPositionOption> startPositionChooser;

    public enum StartPositionOption
    {
        LEFT_START("Hab 1 Left", FieldDimensions.getHab1LeftStartPose()),
        CENTER_LEFT_START("Hab 1 Center (go Left)", FieldDimensions.getHab1CenterLeftStartPose()),
        CENTER_RIGHT_START("Hab 1 Center (go Right)", FieldDimensions.getHab1CenterRightStartPose()),
        RIGHT_START("Hab 1 Right", FieldDimensions.getHab1RightStartPose()),
        HAB2_LEFT_START("Hab 2 Left", FieldDimensions.getHab2LeftStartPose()),
        HAB2_RIGHT_START("Hab 2 Right", FieldDimensions.getHab2RightStartPose());
    	
        public final String name;
        public final Pose initialPose;
   	
        StartPositionOption(String name, Pose initialPose) {
            this.name = name;
            this.initialPose = initialPose;
        }
    }
    	
    public Pose getStartPosition()
    	{
        StartPositionOption startPosition = (StartPositionOption)startPositionChooser.getSelected();
        
		return startPosition.initialPose;
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
   
    public double getStartDelay()
    {
		return startDelayChooser.getSelected().delaySec;
    	}
    




    SendableChooser<FieldDimensions.TargetPositionEnum> firstTargetChooser;
    SendableChooser<FieldDimensions.TargetPositionEnum> secondTargetChooser;
    
    public FieldDimensions.TargetPositionEnum getAutoFirstTarget()
    {
		return firstTargetChooser.getSelected();
    }
    public FieldDimensions.TargetPositionEnum getAutoSecondTarget()
    {
		return secondTargetChooser.getSelected();
}
    
}
   


