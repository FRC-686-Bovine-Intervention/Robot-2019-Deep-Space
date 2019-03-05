package frc.robot.auto.modes;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.SmartDashboardInteractions;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.HatchEjectAction;
import frc.robot.auto.actions.HatchResetAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.auto.modes.FieldDimensions.TargetPositionEnum;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;
import frc.robot.loops.DriveLoop;

/**
 * 2-Hatch Autonomous mode for Sandstorm period
 */

public class HatchAuto extends AutoModeBase {

    public HatchAuto() 
    { 
    }

    @Override
    protected void routine() throws AutoModeEndedException 
    {
        double speed = 48;    //DriveLoop.kPathFollowingMaxVel;
        double visionSpeed = 24;    // slow down for collision/score
        double accel = 24;   //DriveLoop.kPathFollowingMaxAccel
        double lookaheadDist = DriveLoop.kPathFollowingLookahead;

        PathSegment.Options fastOptions =   new PathSegment.Options(speed, accel, lookaheadDist, false);
        PathSegment.Options slowOptions =   new PathSegment.Options(speed, accel, lookaheadDist, false);
        PathSegment.Options medOptions =    new PathSegment.Options(speed, accel, lookaheadDist, false);
        PathSegment.Options visionOptions = new PathSegment.Options(visionSpeed, accel, lookaheadDist, true);
       


        SmartDashboardInteractions smartDashboardInteractions = SmartDashboardInteractions.getInstance();
        double startDelaySec = smartDashboardInteractions.getStartDelay();
        
        TargetPositionEnum target1 = smartDashboardInteractions.getAutoFirstTarget();
        TargetPositionEnum target2 = smartDashboardInteractions.getAutoSecondTarget();
        

        //============================================================================
        // First Target
        //============================================================================

        Pose startPose = smartDashboardInteractions.getStartPosition();
        Vector2d target1StartPos = startPose.getPosition();

        Vector2d target1TurnPos =    FieldDimensions.getTargetTurnPosition(target1);
        Vector2d target1VisionPos =  FieldDimensions.getTargetVisionPosition(target1);
        Vector2d target1HatchPos =   FieldDimensions.getTargetHatchPosition(target1);
        Vector2d target1BackupPos1 = FieldDimensions.getTargetBackupPosition1(target1);
        Vector2d target1BackupPos2 = FieldDimensions.getTargetBackupPosition2(target1);
        Vector2d target1BackupPos3 = FieldDimensions.getTargetBackupPosition3(target1);

        Path firstTargetPathF = new Path();
        firstTargetPathF.add(new Waypoint(target1StartPos,  medOptions));
        firstTargetPathF.add(new Waypoint(target1TurnPos,   medOptions));
        firstTargetPathF.add(new Waypoint(target1VisionPos, visionOptions));
        firstTargetPathF.add(new Waypoint(target1HatchPos,  visionOptions));
        firstTargetPathF.setReverseDirection();

        Path firstTargetPathB = new Path();
        firstTargetPathB.add(new Waypoint(target1HatchPos,   fastOptions));
        firstTargetPathB.add(new Waypoint(target1BackupPos1, fastOptions));
        firstTargetPathB.add(new Waypoint(target1BackupPos2, fastOptions));
        firstTargetPathB.add(new Waypoint(target1BackupPos3, fastOptions));

        //============================================================================
        // Human Station
        //============================================================================

        Vector2d humanStationVisionPos = FieldDimensions.getHumanStationVisionPosition();
        Vector2d humanStationHatchPos =  FieldDimensions.getHumanStationHatchPosition();
        
        // TODO: remember to add a mid-field position to avoid the rocket


        //============================================================================
        // Second Target
        //============================================================================

        Vector2d target2StartPos =   humanStationHatchPos;
        Vector2d target2BackupTurnPos = FieldDimensions.getTargetBackupTurnPosition(target2);
        Vector2d target2TurnPos =       FieldDimensions.getTargetTurnPosition(target2);
        Vector2d target2VisionPos =     FieldDimensions.getTargetVisionPosition(target2);
        Vector2d target2HatchPos =      FieldDimensions.getTargetHatchPosition(target2);
        Vector2d target2BackupPos1 =    FieldDimensions.getTargetBackupPosition1(target2);
        Vector2d target2BackupPos2 =    FieldDimensions.getTargetBackupPosition2(target2);
        Vector2d target2BackupPos3 =    FieldDimensions.getTargetBackupPosition3(target2);



        //============================================================================
        // Actions
        //============================================================================

        Limelight.getCargoInstance().setLEDMode(Limelight.LedMode.kOff);
        Limelight.getHatchInstance().setLEDMode(Limelight.LedMode.kOn);

        
        // At Starrting Position: Go to Target 1
        runAction(new WaitAction(startDelaySec));               // initial delay (optional)
        runAction(new PathFollowerAction(firstTargetPathF));    // drive off platform towards first target

        // At Target 1:  Save position, Place Hatch, then backup from target
        setRobotPosition(target1);
        runAction(new HatchEjectAction()); //eject hatch action
        // backup and retract pistons
        double retractDelay = 0.5;
        Action waitAndRetractAction = new SeriesAction(Arrays.asList(new WaitAction(retractDelay), new HatchResetAction()));
        runAction(new ParallelAction(Arrays.asList(new PathFollowerAction(firstTargetPathB), waitAndRetractAction)));   // reverse away from target

        // Backed up from Target 1: Drive to Human Station

        // At Human Station: Backup to Target 2

        // At Target 1: Save position, Place Hatch, then backup from target

        // Done!

    }

    void setRobotPosition(FieldDimensions.TargetPositionEnum _target)
    {
        RobotState robotState = RobotState.getInstance();
        DriveState driveState = DriveState.getInstance();
        double currentTime = Timer.getFPGATimestamp();
        
        Vector2d currentPosition = FieldDimensions.getRobotPositionAtTarget(_target);
        double currentHeading = driveState.getHeading();

        robotState.setFieldToVehicle(currentTime, new Pose(currentPosition, currentHeading));
    }
}
