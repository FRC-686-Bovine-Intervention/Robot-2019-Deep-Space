package frc.robot.lib.util;

import frc.robot.Constants;
import frc.robot.command_status.DriveCommand;
import frc.robot.command_status.RobotState;
import frc.robot.command_status.VisionStatus;
import frc.robot.lib.util.Kinematics.WheelSpeed;
import frc.robot.loops.DriveLoop;
import frc.robot.subsystems.Drive;

import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Timer;


/**
 * Action for following a path defined by a Path object.
 * 
 * Serially configures a PathFollower object to follow each path 
 */
public class PathFollower 
{ 
	/* Controller can be started in PATH_FOLLOWING or VISION state, as set by _initialState in constructor
	 * If in PATH_FOLLOWING state, it will transition to VISION state when the robot locates a vision target.  
	 * It will only search for vision targets on vision-enabled path segments (set in PathSegmentOptions)
	 * It will not transition back to PATH_FOLLOWING.
	 * Start a new Action to continue following a path moving after this Action has completed.
	 */
	public enum PathVisionState { PATH_FOLLOWING, VISION };
	
	public PathVisionState state;
	
	public Drive drive = Drive.getInstance();
	public RobotState robotState = RobotState.getInstance();
	public VisionStatus visionStatus = VisionStatus.getInstance();
	private Relay ledRelay = LedRelay.getInstance();
	
	Path path;
	
	public Vector2d avgTargetLocation = new Vector2d(0,0);

	public double distanceFromPath;
	public double lookaheadDist;
	public Vector2d lookaheadPoint = new Vector2d();
	public double headingToTarget;		
	
	public double currentTime;

	public Pose currentPose = new Pose();
	public Pose previousPose = new Pose();
	
	// camera pose with respect to robot
	public Pose cameraPose_Robot = new Pose(Constants.kCameraPoseX, Constants.kCameraPoseY, Constants.kCameraPoseTheta);
	
	public double prevDistanceToTargetInches;
	public double prevHeadingToTarget;
	
	public Vector2d targetLocation = new Vector2d(0,0);
	public double distanceToTargetInches;

	public double remainingDistance;

	public double speed;
	public double curvature;

	public WheelSpeed wheelSpeed;
	
	private double prevSpeed;
	private double prevTime;

	
    public PathFollower(Path _path, PathVisionState _initialState) 
    {
        drive = Drive.getInstance();
        path = _path;
        state = _initialState;
    }

    public void start() 
    {
		prevSpeed = robotState.getSpeed();
		prevTime  = -1;		
        remainingDistance = Double.MAX_VALUE;	// make sure we run update() at least once before finishing
    }


    public void update() 
    {
		//---------------------------------------------------
		// Get inputs
		//---------------------------------------------------
		
		// values from camera, normalized to camera's Field of View (-1 to +1) 
		double imageTimestamp    	 = visionStatus.getImageTimestamp();
		double normalizedTargetX 	 = visionStatus.getNormalizedTargetX();
		double normalizedTargetWidth = visionStatus.getNormalizedTargetWidth();

		currentPose = robotState.getLatestFieldToVehicle();		
		currentTime = Timer.getFPGATimestamp();

//FIXME: add timestamp synchronization to timestamps.  Using adjusted current timestamp for now			
imageTimestamp = currentTime - Constants.kCameraLatencySeconds;		// remove camera latency
		
		// calculate target location based on *previous* robot pose
		previousPose = robotState.getFieldToVehicle(imageTimestamp);

		//---------------------------------------------------
		// Process
		//---------------------------------------------------
		wheelSpeed = pathVisionDrive(currentTime, currentPose, previousPose, imageTimestamp, normalizedTargetX, normalizedTargetWidth);	// sets speed, curvature to follow path

		//---------------------------------------------------
		// Output: Send drive control
		//---------------------------------------------------
        drive.setVelocitySetpoint(wheelSpeed);
	}

    
	public WheelSpeed pathVisionDrive(double _currentTime, Pose _currentPose, Pose _previousPose, double _imageTimestamp, double _normalizedTargetX, double _normalizedTargetWidth)
	{
		if (prevTime < 0)				// initial setting of prevTime is important to limit initial acceleration
			prevTime = _currentTime;	// avoid calling Timer.getFPGATimestamp() in this function to allow off-robot testing
		
		//System.out.println("At " + _currentPose + "  Driving to " + path.getSegmentEnd());
		
		remainingDistance = Double.MAX_VALUE;
		double finalSpeed = 0;
		double maxSpeed = 0;
		double maxAccel = 0;
		
		boolean visionEnabledSegment = path.getSegmentVisionEnable(); 
		if (visionEnabledSegment)
			visionDrive(_currentTime, _currentPose, _previousPose, _imageTimestamp, _normalizedTargetX, _normalizedTargetWidth);
		else
			pathDrive(_currentTime, _currentPose);
			
		if (state == PathVisionState.PATH_FOLLOWING)	 
		{
			remainingDistance = path.getRemainingLength();		// TODO: address stopping when past final segment
			finalSpeed = path.getSegmentFinalSpeed();
			maxSpeed = path.getSegmentMaxSpeed();
			maxAccel = path.getSegmentMaxAccel();
		}
		else
		{
			remainingDistance = distanceToTargetInches - Constants.kPegTargetDistanceThresholdFromCameraInches;
			finalSpeed = path.getSegmentFinalSpeed();
			maxSpeed = Constants.kVisionMaxVel;
			maxAccel = Constants.kVisionMaxAccel;
		}
		
		speedControl(_currentTime, remainingDistance, finalSpeed, maxSpeed, maxAccel);

		if (path.getReverseDirection())
		{
			speed = -speed;
			curvature = -curvature;	// TODO: simplify by removing this, and removing flipping heading 180 degrees below?
		}
		
		wheelSpeed = Kinematics.inverseKinematicsFromSpeedCurvature(speed, curvature);
		wheelSpeed.limit(maxSpeed);
		return wheelSpeed;
	}
	
	public WheelSpeed getWheelVelocity() { return getWheelVelocity(); }
	public Path   getPath() { return path; }	// warning: not returning a defensive copy
	public double getDistanceFromPath() { return distanceFromPath; }
	public PathVisionState getPathVisionState() { return state; }

	
	// Drive towards lookahead point on path
	private void pathDrive(double _currentTime, Pose _currentPose)
	{
		//---------------------------------------------------
		// Find Lookahead Point
		//---------------------------------------------------
		distanceFromPath = path.update(_currentPose.getPosition());
		lookaheadPoint = path.getLookaheadPoint(_currentPose.getPosition(), distanceFromPath);
		
		//---------------------------------------------------
		// Find arc to travel to Lookahead Point
		//---------------------------------------------------
		Vector2d robotToTarget = lookaheadPoint.sub(_currentPose.getPosition());
		lookaheadDist = robotToTarget.length();
		headingToTarget = robotToTarget.angle() - _currentPose.getHeading();
		if (path.getReverseDirection())
			headingToTarget -= Math.PI;	// flip robot around
		
		curvature = 2 * Math.sin(headingToTarget) / lookaheadDist;
	}

	
	
	// drive towards vision target (or follow path if no target acquired)
	public void visionDrive(double _currentTime, Pose _currentPose, Pose _previousPose, double _imageTimestamp, double _normalizedTargetX, double _normalizedTargetWidth)
	{
		ledRelay.set(Relay.Value.kOn); 		// turn on LEDs during Vision-enabled segments
		distanceToTargetInches = Double.MAX_VALUE;
		
		// If we get a valid message from the Vision co-processor, update our estimate of the target location
		if (_normalizedTargetWidth > 0) 
		{
			//-----------------------------------------------------
			// Estimate target location based on previous location,
			// to compensate for latency in processing image
			//-----------------------------------------------------
			Pose cameraPose_Field = cameraPose_Robot.transformBy(_previousPose);
			prevDistanceToTargetInches = Constants.kTargetWidthInches / (2.0*_normalizedTargetWidth*Constants.kTangentCameraHalfFOV);
			prevHeadingToTarget = _previousPose.getHeading() + (-_normalizedTargetX*Constants.kCameraHalfFOVRadians);
			Vector2d prevToTarget = Vector2d.magnitudeAngle(prevDistanceToTargetInches, prevHeadingToTarget);
			targetLocation = cameraPose_Field.getPosition().add(prevToTarget); 	
			
			// filter target location with exponential averaging

			if (prevDistanceToTargetInches < Constants.kVisionMaxDistanceInches)	// ignore bogus targets
			{
				if (state == PathVisionState.PATH_FOLLOWING)
				{
					avgTargetLocation = targetLocation;
					state = PathVisionState.VISION;			
				}
				else
				{
					avgTargetLocation = avgTargetLocation.expAverage(targetLocation, Constants.kTargetLocationFilterConstant);
				}
			}
		}
		
		// Drive towards target, even if we didn't get a valid Vision co-processor message this time
		if (state == PathVisionState.VISION)
		{
			Vector2d robotToTarget = avgTargetLocation.sub(_currentPose.getPosition());
			distanceToTargetInches = robotToTarget.length();
			headingToTarget = robotToTarget.angle() - _currentPose.getHeading();
			
			//---------------------------------------------------
			// Calculate motor settings to turn towards target   
			//---------------------------------------------------
			lookaheadDist = Math.min(Constants.kVisionLookaheadDist, distanceToTargetInches);	// length of chord <= kVisionLookaheadDist
			curvature     = 2 * Math.sin(headingToTarget) / lookaheadDist;				// curvature = 1/radius of circle (positive: turn left, negative: turn right)
		}
		else
		{
			// target not acquired -- speed/curvature will be controlled by path follower
			pathDrive(_currentTime, _currentPose);
		}
	}
	
	// keep speed within acceleration limits
	public void speedControl(double _currentTime, double _remainingDistance, double _finalSpeed, double _maxSpeed, double _maxAccel)
	{
		//---------------------------------------------------
		// Apply speed control
		// Note: speed will always be positive in this function
		// it will be made negative before applying to motors
		// if path is reversed
		//---------------------------------------------------
		speed = _maxSpeed;
		
		double dt = _currentTime - prevTime;
		
		// apply acceleration limits
		double accel = (speed - prevSpeed) / dt;
		if (accel > _maxAccel)
			speed = prevSpeed + _maxAccel * dt;
		else if (accel < -_maxAccel)
			speed = prevSpeed - _maxAccel * dt;

		// apply braking distance limits
		// vf^2 = v^2 + 2*a*d   Solve for v, configured vf, a, and measured d
		double stoppingDistance = _remainingDistance;
		double maxBrakingSpeed = Math.sqrt(_finalSpeed * _finalSpeed + 2.0 * _maxAccel * stoppingDistance);
		if (speed > maxBrakingSpeed)
			speed = maxBrakingSpeed;

		// apply minimum velocity limit (Talons can't track low speeds well)
		final double kMinSpeed = 4.0;
		if (speed < kMinSpeed)
			speed = kMinSpeed;

		// store for next time through loop	
		prevTime = _currentTime;
		prevSpeed = speed;
	}
		
	
    public boolean isFinished() 
    {
    	boolean done = false;
    	
    	if (state == PathVisionState.PATH_FOLLOWING)
	        done = (remainingDistance <= DriveLoop.kPathFollowingCompletionTolerance);
    	else
    		done = (remainingDistance <= Constants.kVisionCompletionTolerance);
    	
     	return done;
    }

    public void done() 
    {
		// cleanup code, if any
    	ledRelay.set(Relay.Value.kOff); 		// turn off LEDs when done
    	drive.setVelocitySetpoint(new WheelSpeed(path.finalSpeed, path.finalSpeed));
    }

 
    
    
    private final DataLogger logger = new DataLogger()
    {
        @Override
        public void log()
        {            
			DriveCommand cmd = drive.getCommand();
    		put("PathVision/driveMode", cmd.getDriveControlMode().toString() );
    		put("PathVision/talonMode", cmd.getTalonControlMode().toString() );
    		put("PathVision/left",  cmd.getLeftMotor() );
       		put("PathVision/right", cmd.getRightMotor() );
      		put("PathVision/neutralMode", DriveCommand.getNeutralMode().toString() );
      		
       		Pose odometry = robotState.getLatestFieldToVehicle();
            put("PathVision/positionX",  odometry.getX());
            put("PathVision/positionY",  odometry.getY());
            put("PathVision/headingDeg", odometry.getHeadingDeg());
        	
    		put("PathVision/imageTimestamp", visionStatus.getImageTimestamp() );
    		put("PathVision/normalizedTargetX", visionStatus.getNormalizedTargetX() );
    		put("PathVision/normalizedTargetWidth", visionStatus.getNormalizedTargetWidth() );
            
			put("PathVision/reversed", path.getReverseDirection());
			put("PathVision/state", state.toString());

			put("PathVision/segmentStartX", path.getSegmentStart().getX());
			put("PathVision/segmentStartY", path.getSegmentStart().getY());
			put("PathVision/segmentEndX", path.getSegmentEnd().getX());
			put("PathVision/segmentEndY", path.getSegmentEnd().getY());
			put("PathVision/segmentFinalSpeed", path.getSegmentFinalSpeed());
			put("PathVision/segmentMaxSpeed", path.getSegmentMaxSpeed());
			put("PathVision/segmentVisionEnable", path.getSegmentVisionEnable());
			
			put("PathVision/distanceFromPath", distanceFromPath );
			put("PathVision/lookaheadDist", lookaheadDist );
			put("PathVision/lookaheadPointX",  lookaheadPoint.getX() );
			put("PathVision/lookaheadPointY",  lookaheadPoint.getY());

			put("PathVision/prevPoseX", previousPose.getX());
			put("PathVision/prevPoseY", previousPose.getY());
			
			put("PathVision/targetLocationX", targetLocation.getX());
			put("PathVision/targetLocationY", targetLocation.getY());
			put("PathVision/avgTargetLocationX", avgTargetLocation.getX());
			put("PathVision/avgTargetLocationY", avgTargetLocation.getY());
			put("PathVision/distanceToTargetInches", distanceToTargetInches);
			put("PathVision/headingToTarget", headingToTarget);

			put("PathVision/remainingDistance",  remainingDistance );
			
			put("PathVision/speed", 	speed);
			put("PathVision/curvature", curvature );
			put("PathVision/lSpeed", 	wheelSpeed.left);
			put("PathVision/rSpeed", 	wheelSpeed.right);
			
        }
    };
	
    public DataLogger getLogger() { return logger; }
}
