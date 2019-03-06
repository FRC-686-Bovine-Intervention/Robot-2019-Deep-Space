package frc.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.lib.joystick.ButtonBoard;
import frc.robot.lib.joystick.SelectedJoystick;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.FallingEdgeDetector;
import frc.robot.lib.util.PulseTrain;
import frc.robot.lib.util.RisingEdgeDetector;
import frc.robot.lib.util.Util;
import frc.robot.loops.Loop;

public class CargoIntake implements Loop
{
	// singleton class
    private static CargoIntake instance = null;
    public static CargoIntake getInstance() 
    { 
        if (instance == null) {
            instance = new CargoIntake();
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

    SelectedJoystick selectedJoystick = SelectedJoystick.getInstance();
    public ButtonBoard buttonBoard = ButtonBoard.getInstance();

    public RisingEdgeDetector  intakeButtonRisingEdgeDetector = new RisingEdgeDetector();
    public FallingEdgeDetector intakeButtonFallingEdgeDetector = new FallingEdgeDetector();
    public RisingEdgeDetector onDefenseButtonPress = new RisingEdgeDetector();
    public RisingEdgeDetector onClimbingButtonPress = new RisingEdgeDetector();
    public boolean intakeActive = false;

    public enum CargoDeployStateEnum { ZEROING, OPERATIONAL, CLIMBING; }
    public CargoDeployStateEnum state = CargoDeployStateEnum.ZEROING;

    public enum CargoDeployPositionEnum
    {
        RETRACTED(109.5), 
        HAB_LEVEL3(63.5),
        CARGO_SHIP(56.0), 
        ROCKET(36.0), 
        HAB_LEVEL2(14.0),
        GROUND(-8.7), 
        DEPOT_LEVEL(-2),
        PUSHUP(-14.3),
        LEVEL2_APPROACH(10.0);
        
        public final double angleDeg;
        
        CargoDeployPositionEnum(double _angleDeg) { angleDeg = _angleDeg; }
    }
    public CargoDeployPositionEnum targetPosition = CargoDeployPositionEnum.RETRACTED;
    public double targetAngleDeg = targetPosition.angleDeg;
    
    public boolean zeroed = false;
    
    //====================================================
    // Constants
    //====================================================
    
    public final double kIntakePercentOutput  = +0.6;          
    public final double kOuttakePercentOutput = -1.0;   // full power outtake
    
    public RisingEdgeDetector ballDetectRisingEdge = new RisingEdgeDetector();
    boolean startCargoRetract = false;
    final int kIntakeNumPulses = 4;
    final double kIntakePulseOnTime = 0.10;
    final double kIntakePulseOffTime = 0.25;
    PulseTrain intakePulseTrain = new PulseTrain(kIntakeNumPulses, kIntakePulseOnTime, kIntakePulseOffTime);
    
  
    public final double zeroingPercentOutput = -0.2;

    public final double kMinFwdOutput = +0;
    public final double kMinRevOutput = -0;
    public final double kMaxFwdOutput = +0.5;   // start with low voltage!!!  TODO: increase to 0.5 max (6V)
    public final double kMaxRevOutput = -0.5;

    public final int kSlotIdx = 0;

    public final double kCalMaxEncoderPulsePer100ms = 60;	// velocity at a max throttle (measured using Phoenix Tuner)
    public final double kCalMaxPercentOutput 		= 1.0;	// percent output of motor at above throttle (using Phoenix Tuner)

    public final double kCruiseVelocity = 0.50 * kCalMaxEncoderPulsePer100ms;		// cruise below top speed
    public final double kTimeToCruiseVelocity = 0.25;				// seconds to reach cruise velocity
    public final double kAccel = kCruiseVelocity / kTimeToCruiseVelocity; 
    
	public final double kKf = kCalMaxPercentOutput * 1023.0 / kCalMaxEncoderPulsePer100ms;
	public final double kKp = 32.0;	   
	public final double kKd = 0.0;	// to resolve any overshoot, start at 10*Kp 
	public final double kKi = 0.0;    

	public static double kQuadEncoderGain = 60.0/12.0;			// Arm is on 60t sprocket, Encoder is on 12t sprocket.  
	public static double kQuadEncoderUnitsPerRev = 4*64;
	public static double kEncoderUnitsPerDeg = kQuadEncoderUnitsPerRev * kQuadEncoderGain / 360.0; 

    public final int    kAllowableError = (int)(0.25 * kEncoderUnitsPerDeg);
    public final double kAllowableGroundAngleDeg = CargoDeployPositionEnum.GROUND.angleDeg + 3.0;
    public final double kspinIntakeAngleDeg = 0;

    public final int kPeakCurrentLimit = 50;
    public final int kPeakCurrentDuration = 200;
    public final int kContinuousCurrentLimit = 40;

    public final int kDeployMotorForwardSoftLimit = angleDegToEncoderUnits(CargoDeployPositionEnum.GROUND.angleDeg);
    public final int kDeployMotorReverseSoftLimit = angleDegToEncoderUnits(CargoDeployPositionEnum.RETRACTED.angleDeg);
    

    public static RisingEdgeDetector climbingStartEdgeDetector = new RisingEdgeDetector();

    public CargoIntake() 
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
		deployMotorMaster.setInverted(true);   // set to have green LEDs when driving forward
		
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
        deployMotorMaster.configAllowableClosedloopError(kSlotIdx, kAllowableError, Constants.kTalonTimeoutMs);
        
		// set acceleration and cruise velocity
		deployMotorMaster.configMotionCruiseVelocity((int)kCruiseVelocity, Constants.kTalonTimeoutMs);
		deployMotorMaster.configMotionAcceleration((int)kAccel, Constants.kTalonTimeoutMs);	
        
        // configure talon to automatically reset its position when the reverse limit switch is hit
        deployMotorMaster.configSetParameter(ParamEnum.eClearPositionOnLimitR, 1, 0x00, 0x00, Constants.kTalonTimeoutMs);
        
		// set soft limits (will not be valid until calibration completes)
		deployMotorMaster.configForwardSoftLimitThreshold( kDeployMotorForwardSoftLimit, Constants.kTalonTimeoutMs);
		deployMotorMaster.configReverseSoftLimitThreshold( kDeployMotorReverseSoftLimit, Constants.kTalonTimeoutMs);
        
        // current limits
        deployMotorMaster.configPeakCurrentLimit(kPeakCurrentLimit, Constants.kTalonTimeoutMs);
        deployMotorMaster.configPeakCurrentDuration(kPeakCurrentDuration, Constants.kTalonTimeoutMs);
        deployMotorMaster.configContinuousCurrentLimit(kContinuousCurrentLimit, Constants.kTalonTimeoutMs);
        deployMotorMaster.enableCurrentLimit(true);
        
        
        // configure followers
        deployMotorSlave.follow(deployMotorMaster);
        deployMotorSlave.setInverted(InvertType.OpposeMaster);  // motor is mounted 180 degrees, so set direction opposite of master
 
        // begin with motors stopped
        deployMotorMaster.set(ControlMode.PercentOutput, 0.0);
        deployMotorMaster.setNeutralMode(NeutralMode.Brake);
        
        

        intakeMotor.setInverted(true);

    
        zeroed = false;     // calibrate only once per RoboRIO power cycle
    }
    
	@Override
	public void onStart() 
	{
        // if we haven't calibrated yet, do so now
        if (!zeroed)
        {
            state = CargoDeployStateEnum.ZEROING;

            //turnOffSoftLimits();
        }
    }

    
	@Override
	public void onStop() 
	{
        // stop all motors
        deployMotorMaster.set(ControlMode.PercentOutput, 0.0);
        intakeMotor.set(ControlMode.PercentOutput, 0.0);
    }
    
    boolean drivingCargo = false;
    boolean intakeButton = false;
    boolean intakeButtonPress = false;
    boolean intakeButtonUnpress = false;
    boolean outtakeButton = false;

	@Override
	public void onLoop()
    {
        getLimitSwitches();

        drivingCargo = SelectedJoystick.getInstance().getDrivingCargo();

        intakeButton = selectedJoystick.getButton(Constants.kCargoIntakeButton) && drivingCargo;
        intakeButtonPress = intakeButtonRisingEdgeDetector.update(intakeButton);
        intakeButtonUnpress = intakeButtonFallingEdgeDetector.update(intakeButton);
        outtakeButton = selectedJoystick.getAxisAsButton(Constants.kCargoOuttakeAxis) && drivingCargo;

        runDeploy();
        runIntake();
    }
    

    public void runDeploy()
    {
        climbingStartEdgeDetector.update(buttonBoard.getButton(Constants.kClimbingStartButton));

        switch (state) 
        {
        case ZEROING:
            // on first activation after power-cycling, start moving arm backwards until limit switch is reached
            deployMotorMaster.set(ControlMode.PercentOutput, zeroingPercentOutput);
            
            if (zeroed || getReverseLimitSwitch()) 
            {
                zeroed = true;  // set so we don't have to zero again
                
                // the motor controller position will automatically be set to 0 when we hit the reverse limit switch
                
                turnOnSoftLimits();

                state = CargoDeployStateEnum.OPERATIONAL;
                setTarget(CargoDeployPositionEnum.RETRACTED);
            }
            break;

        case OPERATIONAL:
            // already calibrated, not in climbing mode
            // button board controls are enabled
            runOperational();

            // if (buttonBoard.getButton(Constants.kClimbingStartButton) && (Timer.getMatchTime() < 30))
            if (climbingStartEdgeDetector.get()) // TODO: add code to only allow in last 30 seconds???
            {
                state = CargoDeployStateEnum.CLIMBING;
                Climber.startOver();
            }
            break;

        case CLIMBING:
            // Do nothing in this file -- see Climber.java

            // can get out of climbing mode only by hitting retract button
            if (buttonBoard.getButton(Constants.kDefenseButton))
            {
                state = CargoDeployStateEnum.OPERATIONAL;
                // will go to retracted posistion on next cycle
            }
            break;

        }   // end switch
    }   // end runDeploy()

    
    
    public void runOperational()
    {
        // Only called during OPERATIONAL state
        
        // get current target angle from driver & operator
        if (intakeButtonPress)                                              { setTarget(CargoDeployPositionEnum.GROUND); }      // go to ground on driver button, not operator's button board
        if (selectedJoystick.getButton(Constants.kCargoIntakeDepotHeight))       { setTarget(CargoDeployPositionEnum.DEPOT_LEVEL); }      
        if (buttonBoard.getButton(Constants.kCargoIntakeRocketButton))      { setTarget(CargoDeployPositionEnum.ROCKET); }      // TODO: only allow if ball is detected?
        if (buttonBoard.getButton(Constants.kCargoIntakeCargoShipButton))   { setTarget(CargoDeployPositionEnum.CARGO_SHIP); }  // TODO: only allow if ball is detected?
        if (buttonBoard.getButton(Constants.kDefenseButton))                { setTarget(CargoDeployPositionEnum.RETRACTED); }

        // if in the ground state, turn off motor while riding on wheels
        if ((targetPosition == CargoDeployPositionEnum.GROUND) && (getArmAngleDeg() < kAllowableGroundAngleDeg))
        {
            deployMotorMaster.set(ControlMode.PercentOutput, 0.0);
            deployMotorMaster.setNeutralMode(NeutralMode.Coast);
        }
        
        startCargoRetract = ballDetect() && (getArmAngleDeg() < kspinIntakeAngleDeg);
        if (startCargoRetract)
        {
            // Successful ball intake.  Return to retracted position.  Driver can continue to center ball by holding down intake button.
            setTarget(CargoDeployPositionEnum.RETRACTED);
        }

        
       
        
        startCargoRetract = ballDetect() && (getArmAngleDeg() < kAllowableGroundAngleDeg);
        if (startCargoRetract)
        {
            // Successful ball intake.  Return to retracted position.  Driver can continue to center ball by holding down intake button.
            setTarget(CargoDeployPositionEnum.RETRACTED);
        }
    }

    

    public void runIntake()
    {

        // start intake when button is pressed
        if (intakeButtonPress) { 
            intakeActive = true; }        
        // stop intake when button is unpressed, outtake button is pressed or ball is detected
        if (intakeButtonUnpress || outtakeButton || startCargoRetract)  { 
            intakeActive = false; }       
        // also start pulse train if ball is detected
        if (startCargoRetract) {
             intakePulseTrain.start(); }   
        
        boolean intakePulse = intakePulseTrain.update();
        if (selectedJoystick.getAxisAsButton(Constants.kCargoOuttakeAxis) && drivingCargo) {
            intakeMotor.set(ControlMode.PercentOutput, kOuttakePercentOutput); }
        else if ((getArmAngleDeg() < kspinIntakeAngleDeg) || intakePulse) {
            intakeMotor.set(ControlMode.PercentOutput, kIntakePercentOutput); 
        } 
        else {
            intakeMotor.set(ControlMode.PercentOutput, 0.0); 
        }
    }


    public void setState(CargoDeployStateEnum _state)
    {   
        state = _state;   
    }

    public void setTarget(CargoDeployPositionEnum _targetPosition)
    {
        targetPosition = _targetPosition;
        setTargetAngleDeg(targetPosition.angleDeg);
    }
    
    public void setTargetAngleDeg(double _angleDeg)
    {
        targetAngleDeg = _angleDeg;
        deployMotorMaster.set(ControlMode.MotionMagic, angleDegToEncoderUnits(targetAngleDeg));
        deployMotorMaster.setNeutralMode(NeutralMode.Brake);
    }
    
    public double getTargetAngleDeg()
    {
        return targetAngleDeg;
    }
    
    public void retract()
    {
        setTarget(CargoDeployPositionEnum.RETRACTED);
    }

    public void setPercentOutput(double _out)
    {
        double out = Util.limit(_out,  1.0);
        deployMotorMaster.set(ControlMode.PercentOutput, out);
        deployMotorMaster.setNeutralMode(NeutralMode.Brake);
    }

    private void turnOnSoftLimits() {
        // enable soft limits after zeroing (to keep us from doing a pushup before climbing)
        deployMotorMaster.configReverseSoftLimitEnable(true, Constants.kTalonTimeoutMs);
        deployMotorMaster.configForwardSoftLimitEnable(true, Constants.kTalonTimeoutMs);
        deployMotorMaster.overrideLimitSwitchesEnable(true); // disable soft limit switches during zeroing
    }
    
    public void turnOffSoftLimits() {
        // disable soft limits during zeroing and when doing a pushup during climb
        deployMotorMaster.configReverseSoftLimitEnable(false, Constants.kTalonTimeoutMs);
        deployMotorMaster.configForwardSoftLimitEnable(false, Constants.kTalonTimeoutMs);
        deployMotorMaster.overrideLimitSwitchesEnable(false); // disable soft limit switches during zeroing
    }
    
    Faults faults = new Faults();
    public void getLimitSwitches()
    {
        // called once per loop iteration
        deployMotorMaster.getFaults(faults);
    }

    public boolean getForwardLimitSwitch()
    {
        return faults.ForwardLimitSwitch;
    }
    
    public boolean getReverseLimitSwitch()
    {
        return faults.ReverseLimitSwitch;
    }    
    
    public boolean getForwardSoftLimit()
    {
        return faults.ForwardSoftLimit;  
    }    
    
    public boolean getReverseSoftLimit()
    {
        return faults.ReverseSoftLimit;  
    }    
    
    public double getArmAngleDeg()
    {
        return encoderUnitsToAngleDeg( deployMotorMaster.getSelectedSensorPosition(Constants.kTalonPidIdx) );
    }
    
    public boolean ballDetect()
    {
        return !ballDetectSensor.get();
    }

    public int angleDegToEncoderUnits(double _desiredAngleDeg) 
    {
        // The RETRACTED position is where the encoder is zeroed.  Everything is referenced to it.
        return (int)((CargoDeployPositionEnum.RETRACTED.angleDeg - _desiredAngleDeg) * kEncoderUnitsPerDeg);
    }
    
    public double encoderUnitsToAngleDeg(int _encoderUnits) 
    {
        // The RETRACTED position is where the encoder is zeroed.  Everything is referenced to it.
        return (CargoDeployPositionEnum.RETRACTED.angleDeg - (_encoderUnits / kEncoderUnitsPerDeg));
    }
    
    public boolean shouldBlink(){
        return intakePulseTrain.getEnabled();
    }
    
	private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
            put("CargoIntake/Deploy/targetPosition", targetPosition.toString());
            put("CargoIntake/Deploy/targetAngleDeg", targetAngleDeg);
            put("CargoIntake/Deploy/armAngleDeg", getArmAngleDeg());
            put("CargoIntake/Deploy/state", state.toString());
            put("CargoIntake/Deploy/zeroed", zeroed);
            put("CargoIntake/Deploy/fwdLimitSwitch", getForwardLimitSwitch());
            put("CargoIntake/Deploy/revLimitSwitch", getReverseLimitSwitch());
            put("CargoIntake/Deploy/fwdSoftLimit", getForwardSoftLimit());
            put("CargoIntake/Deploy/revSoftLimit", getReverseSoftLimit());
            put("CargoIntake/Deploy/motorCurrent", deployMotorMaster.getOutputCurrent());
            put("CargoIntake/Deploy/pidError", deployMotorMaster.getClosedLoopError(kSlotIdx));
            put("CargoIntake/Intake/voltage", intakeMotor.getMotorOutputPercent());
            put("CargoIntake/Intake/ballDetect", ballDetectSensor.get());
		}
	};
    
	public DataLogger getLogger()
	{
		return logger;
	}    
}