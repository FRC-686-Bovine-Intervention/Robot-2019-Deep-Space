package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.GoalStates;
import frc.robot.command_status.GoalStates.GoalState;
import frc.robot.lib.util.DataLogger;
import frc.robot.lib.util.Kinematics;
import frc.robot.lib.util.Kinematics.WheelSpeed;
import frc.robot.loops.DriveLoop;

public class VisionDriveAssistant
{
	private static VisionDriveAssistant instance = new VisionDriveAssistant();	
	public static VisionDriveAssistant getInstance() { return instance; }

    // configuration parameters
    public static boolean allowSpeedControl = false;
    public static double kLookaheadDist = 24.0;   // inches
    public static double kFullThrottleSpeed =      DriveLoop.encoderUnitsPerFrameToInchesPerSecond((int)Constants.kFullThrottleEncoderPulsePer100ms); // inches/sec
    public static double kMaxSpeed =      100.0; // inches/sec
    public static double kMaxAccel =      200.0; // inches/sec^2	
    public static double kTargetStoppingDistanceFromBumper = 16.0; // inches to stop from target, measured from front bumper

    // members
    public GoalStates goalStates = GoalStates.getInstance();
    public boolean enabled;
    public boolean foundTarget;
    public double distanceToTargetInches;
	public double bearingToTarget;
	public double lookaheadDist;
    public double curvature;
    public double joystickSpeed;    // speed set by driver (manual)
    public double approachSpeed;    // speed set by driver or auto calculated
    public double maxSpeed;         // speed limit based on distance from target
    public WheelSpeed wheelSpeed = new WheelSpeed();



    public VisionDriveAssistant() {}

    public DriveCommand assist(DriveCommand _driveCmd, boolean _enable) 
    {
        DriveCommand driveCmd = _driveCmd;
        enabled = _enable;
        
        Optional<GoalState> optGoalState = goalStates.getBestVisionTarget();
        foundTarget = optGoalState.isPresent();

        // if we don't see a target, continue under driver control
        if (foundTarget)
        {
            // Get range and angle to target
            GoalState goalState = optGoalState.get();
            distanceToTargetInches = goalState.getHorizontalDistance() - Constants.kCenterToFrontBumper;   // distance from front bumper
            bearingToTarget = goalState.getRelativeBearing();

            // Calculate motor settings to turn towards target
            lookaheadDist = Math.min(kLookaheadDist, distanceToTargetInches);	// length of chord <= kLookaheadDist
            curvature     = 2 * Math.sin(bearingToTarget) / lookaheadDist;		// curvature = 1/radius of circle (positive: turn left, negative: turn right)

            // get speed limit based on distance            
            double currentTime = Timer.getFPGATimestamp();
            double remainingDistance = distanceToTargetInches - kTargetStoppingDistanceFromBumper;
            maxSpeed = calcSpeedLimit(currentTime, remainingDistance, kMaxSpeed, kMaxAccel);
            maxSpeed = maxSpeed / kFullThrottleSpeed;   // convert from inches/sec to percentage

            // Get forward speed
            joystickSpeed = driveCmd.getSpeed(); // in percentage
            approachSpeed = joystickSpeed;

            if (allowSpeedControl)
            {
                // automatically reduce speed to stop in front of target
                // approachSpeed = Math.min(approachSpeed, maxSpeed);
                approachSpeed = maxSpeed;
            }   

            // calculate left/right motor speeds for this approach speed & curvature
            wheelSpeed = Kinematics.inverseKinematicsFromSpeedCurvature(approachSpeed, curvature);

            if (enabled)
            {
                // adjust drive command
                driveCmd.setMotors(wheelSpeed);
            }
        }
        else
        {
            // set some default values for logging
            distanceToTargetInches = 0.0;
            bearingToTarget = 0.0;
            lookaheadDist = kLookaheadDist;	
            curvature     = 0.0;		
            maxSpeed = 0.0;   
            joystickSpeed = driveCmd.getSpeed(); 
            approachSpeed = joystickSpeed;
        }

		//---------------------------------------------------
		// Output: Send drive control
		//---------------------------------------------------
        return driveCmd;
    }
    

    double prevTime = 0.0;
    double prevSpeed = 0.0;
    // keep speed within acceleration limits
	public double calcSpeedLimit(double _currentTime, double _remainingDistance, double _maxSpeed, double _maxAccel)
	{
		//---------------------------------------------------
		// Apply speed control
		//---------------------------------------------------
		double speed = _maxSpeed;
		
		double dt = _currentTime - prevTime;
		
		// apply acceleration limits
		double accel = (speed - prevSpeed) / dt;
		if (accel > _maxAccel)
			speed = prevSpeed + _maxAccel * dt;
		else if (accel < -_maxAccel)
			speed = prevSpeed - _maxAccel * dt;

		// apply braking distance limits
		// vf^2 = v^2 + 2*a*d   Solve for v, given vf=0, configured a, and measured d
		double stoppingDistance = _remainingDistance;
		double maxBrakingSpeed = Math.sqrt(2.0 * _maxAccel * stoppingDistance);
		if (Math.abs(speed) > maxBrakingSpeed)
			speed = Math.signum(speed) * maxBrakingSpeed;

		// apply minimum velocity limit (Talons can't track low speeds well)
		final double kMinSpeed = 4.0;
		if (Math.abs(speed) < kMinSpeed) 
			speed = Math.signum(speed) * kMinSpeed;

		// store for next time through loop	
		prevTime = _currentTime;
        prevSpeed = speed;
        
        return speed;
    }
    

    private final DataLogger logger = new DataLogger()
	{
		@Override
		public void log()
		{
            put("DriveAssist/enabled", enabled);
            put("DriveAssist/foundTarget", foundTarget);
            put("DriveAssist/distanceToTargetInches", distanceToTargetInches);
            put("DriveAssist/bearingToTarget", bearingToTarget);
            put("DriveAssist/lookaheadDist", lookaheadDist);
            put("DriveAssist/curvature", curvature);
            put("DriveAssist/joystickSpeed", joystickSpeed);
            put("DriveAssist/approachSpeed", approachSpeed);
            put("DriveAssist/maxSpeed", maxSpeed);
            put("DriveAssist/leftWheelSpeed", wheelSpeed.left);
            put("DriveAssist/rightWheelSpeed", wheelSpeed.right);
        }
    };

	public DataLogger getLogger()
	{
		return logger;
	}
}