package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.lib.joystick.ArcadeDriveJoystick;
import frc.robot.lib.joystick.ButtonBoard;
import frc.robot.lib.joystick.JoystickControlsBase;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.loops.Loop;

public class CargoBallIntake implements Loop
{
	// singleton class
    private static CargoBallIntake instance = null;
    public static CargoBallIntake getInstance() 
    { 
        if (instance == null) {
            instance = new CargoBallIntake();
        }
        return instance;
    }

    //====================================================
    // Members
    //====================================================
    public TalonSRX deployMotorMaster;
    public VictorSPX deployMotorSlave;
    public VictorSPX intakeMotor;
    public DigitalInput ballDetectSensor;

    public JoystickControlsBase driverJoystick = ArcadeDriveJoystick.getInstance(); // this control is user-selectable at teleopInit, so we need to update
    public ButtonBoard buttonBoard = ButtonBoard.getInstance();

    public RisingEdgeDetector onIntakeButtonPress;
    public RisingEdgeDetector onDefenseButtonPress;
    public RisingEdgeDetector onClimbingButtonPress;
    public boolean intakeActive = false;

    public enum CargoDeployStateEnum { ZEROING, OPERATIONAL, DEFENSE, CLIMBING; }
    public CargoDeployStateEnum state = CargoDeployStateEnum.ZEROING;

    public enum CargoDeployPositionEnum
    {
        RETRACTED(120.0), 
        GROUND(0.0), 
        CARGO_SHIP(45.0), 
        ROCKET(60.0), 
        PLATFORM(90.0),
        PUSHUP(-10.0); 
    
        public final double angleDeg;

        CargoDeployPositionEnum(double _angleDeg) { angleDeg = _angleDeg; }
    }
    public CargoDeployPositionEnum position = CargoDeployPositionEnum.RETRACTED;
    
    public boolean calibrated = false;

    //====================================================
    // Constants
    //====================================================
    
    public final double zeroingPercentOutput = -0.1;

    public final double IntakeSpeed = 1; 
    public final double OuttakeSpeed = -1; 
    public final double cargoBallStop = 0;

    public final double kMinFwdOutput = +0;
    public final double kMinRevOutput = -0;
    public final double kMaxFwdOutput = +0.1;   // start with low voltage!!!
    public final double kMaxRevOutput = -0.1;

    public final int kSlotIdx = 0;

    public final double kMaxEncoderPulsePer100ms = 1;	// velocity at a max throttle (measured using Phoenix Tuner)
    public final double kMaxPercentOutput 		= 0.1;		// percent output of motor at above throttle (using Phoenix Tuner)

    public final double kCruiseVelocity = 0.75 * kMaxEncoderPulsePer100ms;		// cruise below top speed
    public final double kTimeToCruiseVelocity = 0.25;				// seconds to reach cruise velocity
    public final double kAccel = kCruiseVelocity / kTimeToCruiseVelocity; 
    
	public final double kKf = kMaxPercentOutput * 1023.0 / kMaxEncoderPulsePer100ms;
	public final double kKp = 0.4;	   
	public final double kKd = 0.0;	// to resolve any overshoot, start at 10*Kp 
	public final double kKi = 0.0;    

	public static double kQuadEncoderGain = 60.0/12.0;			// Encoder is on 12t sprocket.  Arm is on 60t sprocket
	public static double kQuadEncoderUnitsPerRev = 64;
	public static double kEncoderUnitsPerDeg = kQuadEncoderUnitsPerRev / 360.0; 

    public final int    kAllowableError = (int)(1.0 * kEncoderUnitsPerDeg);
    
    
    
    public CargoBallIntake() 
    {
        deployMotorMaster = new TalonSRX(Constants.kCargoDeployMasterTalonId);
        deployMotorSlave = new VictorSPX(Constants.kCargoDeploySlaveTalonId);
        intakeMotor = new VictorSPX(Constants.kCargoIntakeTalonId);
        ballDetectSensor = new DigitalInput(Constants.kBallDetectSensorPort);

        //====================================================
        // Configure Deploy Motors
        //====================================================

        // Factory default hardware to prevent unexpected behavior
        deployMotorMaster.configFactoryDefault();

		// configure encoder
		deployMotorMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, Constants.kTalonPidIdx, Constants.kTalonTimeoutMs);
		deployMotorMaster.setSensorPhase(true); // set so that positive motor input results in positive change in sensor value
		deployMotorMaster.setInverted(false);   // set to have green LEDs when driving forward
		
		// set relevant frame periods to be at least as fast as periodic rate
		deployMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General,      (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		deployMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0,    (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		deployMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
		deployMotorMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0,  (int)(1000 * Constants.kLoopDt), Constants.kTalonTimeoutMs);
        
        // set min and max outputs
        deployMotorMaster.configNominalOutputForward(kMinFwdOutput, Constants.kTalonTimeoutMs);
        deployMotorMaster.configNominalOutputReverse(kMinRevOutput, Constants.kTalonTimeoutMs);
        deployMotorMaster.configPeakOutputForward(kMaxFwdOutput, Constants.kTalonTimeoutMs);
        deployMotorMaster.configPeakOutputReverse(kMaxRevOutput, Constants.kTalonTimeoutMs);
		
		// configure position loop PID 
        deployMotorMaster.selectProfileSlot(kSlotIdx, Constants.kTalonPidIdx); 
        deployMotorMaster.config_kF(kSlotIdx, kKf, Constants.kTalonTimeoutMs); 
        deployMotorMaster.config_kP(kSlotIdx, kKp, Constants.kTalonTimeoutMs); 
        deployMotorMaster.config_kI(kSlotIdx, kKi, Constants.kTalonTimeoutMs); 
        deployMotorMaster.config_kD(kSlotIdx, kKd, Constants.kTalonTimeoutMs);
        
		// set acceleration and cruise velocity
		deployMotorMaster.configMotionCruiseVelocity((int)kCruiseVelocity, Constants.kTalonTimeoutMs);
		deployMotorMaster.configMotionAcceleration((int)kAccel, Constants.kTalonTimeoutMs);	
        
        deployMotorMaster.configAllowableClosedloopError(kSlotIdx, kAllowableError, Constants.kTalonTimeoutMs);
                
        deployMotorMaster.set(ControlMode.PercentOutput, 0.0);
        deployMotorMaster.setNeutralMode(NeutralMode.Brake);
        
        // configure followers
        deployMotorSlave.follow(deployMotorMaster);
        deployMotorSlave.setInverted(InvertType.OpposeMaster);  // motor is mounted 180 degrees, so set direction opposite of master

        // set soft limits during normal operation (remove fwd soft limit during climb)
        // TODO

        // set current limits
        // TODO

        calibrated = false;     // calibrate only once per RoboRIO power cycle
    }
    
	@Override
	public void onStart() 
	{
        // if we haven't calibrated yet, do so now
        if (!calibrated)
        {
            state = CargoDeployStateEnum.ZEROING;
        }
	}

	@Override
	public void onStop() 
	{
        // stop all motors
        deployMotorMaster.set(ControlMode.PercentOutput, 0.0);
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }

	@Override
	public void onLoop()
    {
        // update joystick class, in case it was changed in teleopInit
        driverJoystick = Robot.getInstance().getJoystick();

        runDeploy();
        runIntake();
    }

    public void runIntake()
    {
        // if (onIntakeButtonPress.update(driverJoystick.getButton(Constants.kCargoIntakeButton)) 
        // {
        //     // toggle intake every time intake button is pressed
        //     intakeActive = !intakeActive;
        // }

        // if (intakeActive)
        if (driverJoystick.getButton(Constants.kCargoIntakeButton))
        {
            intakeMotor.set(ControlMode.PercentOutput, IntakeSpeed);
            // TODO: currentClosedLoop mode at 20A?
        }

        if (driverJoystick.getButton(Constants.kCargoOuttakeButton)) 
        {
            intakeMotor.set(ControlMode.PercentOutput, OuttakeSpeed);
        }
    }

    public void runDeploy()
    {
        boolean defenseButtonPress = onDefenseButtonPress.update(buttonBoard.getButton(Constants.kDefenseButton));
        boolean climbingButtonPress = onClimbingButtonPress.update(buttonBoard.getButton(Constants.kClimbingStartButton));
        
        switch (state) 
        {
            case ZEROING:
            // on first activation after power-cycling, start moving arm backwards until limit switch is reached
            deployMotorMaster.set(ControlMode.PercentOutput, zeroingPercentOutput);
            if (getReverseLimitSwitch()) 
            {
                calibrated = true;  // set so we don't have to zero again
                state = CargoDeployStateEnum.OPERATIONAL;
            }
            break;
 
        case OPERATIONAL:
            // already calibrated, not in defense mode
            // button board controls are enabled
            runOperational();

            if (defenseButtonPress)
            {
                state = CargoDeployStateEnum.DEFENSE;
            }
            
            if (climbingButtonPress)   // TODO: add code to only allow in last 30 seconds???
            {
                state = CargoDeployStateEnum.CLIMBING;
            }
            break;

        case DEFENSE:
            setPosition(CargoDeployPositionEnum.RETRACTED);
            if (defenseButtonPress) 
            {
                state = CargoDeployStateEnum.OPERATIONAL;
                // will go back to position prior to defense button being pressed the first time
            }
            break;
            
        case CLIMBING:
            // Do nothing in this file -- see Climber.java

            if (climbingButtonPress)  // TODO: only allow during first step of climb???
            {
                state = CargoDeployStateEnum.OPERATIONAL;
                // will go back to position prior to climbing button being pressed the first time
            }
            break;
            
        }
    }



    public void runOperational()
    {
        // Only called during OPERATIONAL state

        // get current target angle from driver & operator
        if (driverJoystick.getButton(Constants.kCargoIntakeButton))         { position = CargoDeployPositionEnum.GROUND; }      // go to ground on driver button, not operator's button board
        if (buttonBoard.getButton(Constants.kCargoIntakeRetractButton))     { position = CargoDeployPositionEnum.RETRACTED; }
        if (buttonBoard.getButton(Constants.kCargoIntakeRocketButton))      { position = CargoDeployPositionEnum.ROCKET; }      // TODO: only allow if ball is detected?
        if (buttonBoard.getButton(Constants.kCargoIntakeCargoShipButton))   { position = CargoDeployPositionEnum.CARGO_SHIP; }  // TODO: only allow if ball is detected?
        
        if ((position == CargoDeployPositionEnum.GROUND) && ballDetectSensor.get())
        {
            // Successful ball intake.  Return to retracted position.  Driver can continue to center ball by holding down intake button.
            position = CargoDeployPositionEnum.RETRACTED;
        }

        setPosition(position);
    }

    public void setPosition(CargoDeployPositionEnum _position)
    {
        deployMotorMaster.set(ControlMode.MotionMagic, angleDegToEncoderUnits(_position.angleDeg));
    }


    public double angleDegToEncoderUnits(double angleDeg) {
        // to do: write this function
        return 0.0;
    }

    public boolean getReverseLimitSwitch()
    {
        Faults faults = new Faults();
        deployMotorMaster.getFaults(faults);
        return faults.ReverseLimitSwitch;
    }

    public boolean getForwardSoftLimit()
    {
        Faults faults = new Faults();
        deployMotorMaster.getFaults(faults);
        return faults.ForwardSoftLimit;  
    }

    public boolean getForwardLimitSwitch()
    {
        Faults faults = new Faults();
        deployMotorMaster.getFaults(faults);
        return faults.ForwardLimitSwitch;
    }

    public void turnOffSoftLimits()
    {
        // turn off soft limits so we can do a pushup during climb
        deployMotorMaster.overrideLimitSwitchesEnable(false);    
    }
}