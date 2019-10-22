package frc.robot.lib.joystick;

public class Thrustmaster extends JoystickBase
{
    public static int kXAxis =              0;
    public static int kYAxis =              1;
    public static int kZRotateAxis =        2;
    public static int kSliderAxis =         3;
    
    public static int kTriggerButton =      1;
    public static int kBottomThumbButton =  2;
    public static int kLeftThumbButton =    3;
    public static int kRightThumbButton =   4;

    // counting from inner (or thumb) side
    public static int kTop1Button =		    5;         
    public static int kTop2Button =		    6;
    public static int kTop3Button =		    7;
    public static int kBottom3Button =		8;
    public static int kBottom2Button =		9;
    public static int kBottom1Button =		10;
    public static int kTop6Button =		    11;
    public static int kTop5Button =		    12;
    public static int kTop4Button =		    13;
    public static int kBottom4Button =		14;
    public static int kBottom5Button =		15;
    public static int kBottom6Button =      16;
    
    // constructor
    public Thrustmaster(int _port)
    {
        super(_port);
    }
}