package frc.robot.lib.util;

/**
 * Rising Edge Detector. 
 * <p>
 * Returns true only when input changes from false to true.
 */
public class RisingEdgeDetector
{
    private boolean lastValue = false;

    public boolean update(boolean newValue) 
    {
        boolean rv = (newValue && !lastValue);
        lastValue = newValue;
        return rv;
    }
}