package frc.robot.lib.joystick;

// A list of all driver controls to be mapped to joystick buttons

public enum DriverControlsEnum {    // Controls Description
    VISION_ASSIST,                  // enable vision assistance
    HATCH_DEPLOY,                   // extend hatch mechanism, grab hatch panel when released 
    HATCH_SHOOT,                    // extend hatch mechanism, release hatch panel with released
    CARGO_INTAKE,                   // move cargo intake to ground (will automatically start intake rollers when close to ground)
    CARGO_OUTTAKE,                  // spin intake rollers to shoot cargo
    CARGO_INTAKE_DEPOT_HEIGHT,      // move cargo intake to depot height (to intake balls without getting stuck on depot)
    CARGO_INTAKE_ROCKET_HEIGHT,     // move cargo intake to rocket height
    CARGO_INTAKE_CARGO_HEIGHT,      // move cargo intake to cargo ship height
    DEFENSE,                        // move cargo intake to defense position
    CLIMB_PREPARE,                  // move cargo intake to HAB level (press multiple times to cycle HAB3, HAB2, DEFENSE)
    CLIMB_EXTEND,                   // extend cylinders, climb
    CLIMB_RETRACT,                  // retract cylinders after successful climb
    EMERGENCY_ZEROING,
    QUICK_TURN                      // to make TriggerDrive joysticks happy
}

