package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.modes.FieldDimensions;
import frc.robot.auto.modes.HatchAutoChamps;
import frc.robot.auto.modes.StandStillMode;
import frc.robot.lib.joystick.*;
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
    	driverControlsChooser = new SendableChooser<DriverControlsOption>();
    	driverControlsChooser.addOption(DriverControlsOption.XBOX_REVERSIBLE_ARCADE.name,        DriverControlsOption.XBOX_REVERSIBLE_ARCADE);
    	driverControlsChooser.addOption(DriverControlsOption.MICHAEL.name,        DriverControlsOption.MICHAEL);
        driverControlsChooser.setDefaultOption(DriverControlsOption.THRUSTMASTER_REVERSIBLE_ARCADE.name,  DriverControlsOption.THRUSTMASTER_REVERSIBLE_ARCADE);
    	// driverControlsChooser.addOption(DriverControlsOption.ARCADE.name,        DriverControlsOption.ARCADE);
		// driverControlsChooser.addOption(DriverControlsOption.TRIGGER.name,        DriverControlsOption.TRIGGER);
    	// driverControlsChooser.addOption(DriverControlsOption.TANK.name, 	      DriverControlsOption.TANK);
     	// driverControlsChooser.addOption(DriverControlsOption.CHEESY_ARCADE.name,  DriverControlsOption.CHEESY_ARCADE);
    	// driverControlsChooser.addOption(DriverControlsOption.CHEESY_TRIGGER.name, DriverControlsOption.CHEESY_TRIGGER);
        // driverControlsChooser.addOption(DriverControlsOption.CHEESY_2STICK.name,  DriverControlsOption.CHEESY_2STICK);
        // driverControlsChooser.addOption(DriverControlsOption.THRUSTMASTER_ARCADE.name,  DriverControlsOption.THRUSTMASTER_ARCADE);
        // driverControlsChooser.addOption(DriverControlsOption.THRUSTMASTER_TANK.name,  DriverControlsOption.THRUSTMASTER_TANK);
        // driverControlsChooser.addOption(DriverControlsOption.THRUSTMASTER_2STICK.name,  DriverControlsOption.THRUSTMASTER_2STICK);
    	SmartDashboard.putData("Driver Controls", driverControlsChooser);



        autoModeChooser = new SendableChooser<AutoModeOption>();
        autoModeChooser.setDefaultOption(AutoModeOption.HATCH_AUTO.name, AutoModeOption.HATCH_AUTO);
        autoModeChooser.addOption(AutoModeOption.STAND_STILL.name, AutoModeOption.STAND_STILL);
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

        
    
    
    SendableChooser<DriverControlsOption> driverControlsChooser;
    
    enum DriverControlsOption 
    {
        XBOX_REVERSIBLE_ARCADE("Xbox Reversible Arcade"),
        THRUSTMASTER_REVERSIBLE_ARCADE("Thrustmaster Reversible Arcade"),
        MICHAEL("Michael");
        // ARCADE("Arcade"),
        // TRIGGER("Trigger"),				// works for Xbox controller and Xbox steering wheel
        // TANK("Tank"),
        // CHEESY_ARCADE("Cheesy Arcade"),
        // CHEESY_TRIGGER("Cheesy Trigger"),
        // CHEESY_2STICK("Cheesy Two-Stick"),
        // THRUSTMASTER_ARCADE("Thrustmaster Arcade"),
        // THRUSTMASTER_TANK("Thrustmaster Tank"),
        // THRUSTMASTER_2STICK("Thrustmaster Two-Stick");

    	public final String name;
    	
        DriverControlsOption(String name) {
    		this.name= name;
    	}
    }
   
    public ReversibleDriverControlsBase getDriverControlsSelection() 
    {
    	DriverControlsOption selection = (DriverControlsOption)driverControlsChooser.getSelected(); 
    
    	switch (selection)
    	{
		case XBOX_REVERSIBLE_ARCADE:
           return new DriverControlsReversibleXbox();

        case THRUSTMASTER_REVERSIBLE_ARCADE:
        default:
            return new DriverControlsReversibleThrustmaster(); 

        case MICHAEL:
            return new DriverControlsMichael();

}   
    }
    
    

    	
    static SendableChooser<AutoModeOption> autoModeChooser;
    
    enum AutoModeOption
    {
        HATCH_AUTO("Hatch Panel Auto"),
        STAND_STILL("Stand Still");
    
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
   


