package frc.robot.lib.util;

/**
 * A class that stores the pose an object
 * The pose consists of it's position: (x,y) coordinates
 * and it's orientation: the heading
 * The reference coordinate system for the pose is not defined in this class.  The caller must keep track of it.
 */
public class Pose implements Interpolable<Pose>
{
    private Vector2d position;	// position (x,y) in inches
    private double heading;		// heading in radians

    // constructors
    public Pose() 
    {
    	this(0, 0, 0);
    }

    public Pose(double _x, double _y) 
    {
    	this(_x, _y, 0.0);
    }

    public Pose(double _x, double _y, double _heading) 
    {
        position = new Vector2d(_x,_y);
        heading  = _heading;
    }

    public Pose(Vector2d _position, double _heading) 
    {
    	position = new Vector2d(_position);
		heading  = _heading;
    }

    public Pose(Pose that) 
    {
    	this(that.position, that.heading);
    }

    
	/** multiply by this to convert from radians to degrees */
	static public final double radiansToDegrees = 180 / Math.PI;
	static public final double radDeg = radiansToDegrees;
	/** multiply by this to convert from degrees to radians */
	static public final double degreesToRadians = Math.PI / 180;
	static public final double degRad = degreesToRadians;
    
    public static Pose fromMagnitudeAngle(double _rho, double _theta)
    {
    	Vector2d p = new Vector2d(_rho, 0);
    	p.rotate(_theta);
    	return new Pose(p, _theta);		// arbitrarily setting heading to theta
    }
    
    public double getX() { return position.x; }
    public double getY() { return position.y; }
    public Vector2d getPosition() { return position; }
    public double getHeading() { return heading; }
    public double getHeadingDeg() { return heading * radiansToDegrees; }
    public Vector2d getHeadingUnitVector() { return new Vector2d(Math.cos(heading), Math.sin(heading)); }

    // for when the Pose represents a translation/rotation
    public Vector2d getTranslation() { return position; }
    public double getRotation() { return heading; }
    public double getRotationDeg() { return heading * radiansToDegrees; }

    
    // add performs vector translation.  The original heading is not changed
    public Pose add(Vector2d _translation)
    {
    	return new Pose(position.add(_translation), heading);
    }
    
    // returns vector from that to this.position
    public Vector2d sub(Vector2d _translation)
    {
    	return position.sub(_translation);
    }
    
    // returns vector from that.position to this.position
    public Vector2d sub(Pose _that)
    {
    	return this.sub(_that.position);
    }
    
    // adjust heading without changing position
    // (use rotate to rotate about origin)
    public Pose turn(double _theta)
    {
    	return new Pose(position, heading+_theta);
    }
    
    // rotates position about origin, and adjusts heading by _theta
    public Pose rotate(double _theta)
    {
    	return new Pose(position.rotate(_theta), heading+_theta);
    }

    // get distance from this pose to vector v
    public double distance(Vector2d _that)
    {
    	return position.distance(_that);
    }

    // heading from this to that in radians
    public double heading(Vector2d _that)
    {
    	return this.position.angle(_that);
    }

    // heading from this to that in degrees
    public double headingDeg(Vector2d _that)
    {
    	return heading(_that) * radiansToDegrees;
    }
    
    
    
     // Linear interpolation of poses
    @Override
    public Pose interpolate(Pose _that, double _u)
    {
    	double u = _u;
        if (u < 0)
            u = 0;	
        if (u > 1) 
            u = 1;
        
    	Vector2d iPosition = position.interpolate(_that.position, u);		// interpolate position
    	double  iHeading = this.heading + u*(_that.heading - this.heading);	// interpolate heading
    	 
        return new Pose(iPosition, iHeading);
    }

    
    
    @Override
    public String toString() 
    {
    	return String.format("%s, H: % 5.1f deg", position.toString(), getHeadingDeg());
    }

    // transforms pose from one coordinate system to another coordinate system
    // that gives the position and heading of the first coordinate system with respect to that's coordinate system
    // for example: the pose of the camera on the robot is found relative to the field
    //              this would be the pose of the camera with respect to the robot's center of rotation and heading
    //              that would give the pose of the robot with respect to the field
	public Pose transformBy(Pose _that)
	{
																// assume robot's center of rotation is (0,0)
		Pose transformedPose = this.rotate(_that.heading);	// first, rotate by that.heading
		transformedPose = transformedPose.add(_that.position);	// then translate by that.position
		return transformedPose;
	}
	
	// Returns the inverse transform
    // The inverse of this transform "undoes" the effect of translating by this transform.
    public Pose inverse()
    {
    	Pose T = new Pose(-this.getTranslation().x, -this.getTranslation().y, 0);	// invert translation
    	T = T.rotate(-this.getRotation());								// rotate translation by inverse of rotation
    	return T;												
    }
	
}


