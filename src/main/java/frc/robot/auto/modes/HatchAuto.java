package frc.robot.auto.modes;

import java.util.Arrays;

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


        PathSegment.Options pathOptions	=       new PathSegment.Options(speed, accel, lookaheadDist, false);
        PathSegment.Options tightTurnOptions =  new PathSegment.Options(speed, accel, lookaheadDist/2, false);
        PathSegment.Options visionOptions =     new PathSegment.Options(visionSpeed, accel, lookaheadDist, true);
        PathSegment.Options fastOptions =     new PathSegment.Options(speed, accel, lookaheadDist, false);
        PathSegment.Options medOptions =     new PathSegment.Options(speed, accel, lookaheadDist, false);
        PathSegment.Options slowOptions =     new PathSegment.Options(speed, accel, lookaheadDist, false);
       


        SmartDashboardInteractions smartDashboardInteractions = SmartDashboardInteractions.getInstance();
        Pose startPose = smartDashboardInteractions.getStartPosition();
        Vector2d startPosition = startPose.getPosition();

        double startDelaySec = smartDashboardInteractions.getStartDelay();

        TargetPositionEnum target[] = new TargetPositionEnum[2];
        target[0] = smartDashboardInteractions.getAutoFirstTarget();
        target[1] = smartDashboardInteractions.getAutoSecondTarget();

        Path firstTargetPathB = new Path();
        firstTargetPathB.add(new Waypoint(target1HatchPos,   fastOptions));
        firstTargetPathB.add(new Waypoint(target1BackupPos1, fastOptions));
        firstTargetPathB.add(new Waypoint(target1BackupPos2, fastOptions));
        firstTargetPathB.add(new Waypoint(target1BackupPos3, fastOptions));

        //============================================================================
        // Human Station
        //============================================================================

        Vector2d humanStationTurnPos =   FieldDimensions.getHumanStationTurnPosition();
        Vector2d humanStationVisionPos = FieldDimensions.getHumanStationVisionPosition();
        Vector2d humanStationHatchPos =  FieldDimensions.getHumanStationHatchPosition();
        
        Path humanStationPathF = new Path();
        humanStationPathF.add(new Waypoint(target1BackupPos3, fastOptions));
        if(target1 == FieldDimensions.TargetPositionEnum.ROCKET_FAR){
            humanStationPathF.add(new Waypoint(FieldDimensions.getHumanStationFarRocketMidPosition(), fastOptions));
        }
        if (target1 == FieldDimensions.TargetPositionEnum.CARGO_SIDE1 ||
        target1 == FieldDimensions.TargetPositionEnum.CARGO_SIDE2 || 
        target1 == FieldDimensions.TargetPositionEnum.CARGO_SIDE3 );
        {
            humanStationPathF.add(new Waypoint(FieldDimensions.getHumanStationSideCargoMidPosition(), fastOptions));
        }
        humanStationPathF.add(new Waypoint(humanStationTurnPos, fastOptions));
        humanStationPathF.add(new Waypoint(humanStationVisionPos, visionOptions));
        humanStationPathF.add(new Waypoint(humanStationHatchPos, visionOptions));
        humanStationPathF.setReverseDirection();

           // Vector2d backupPos = FieldDimensions.getTargetBackupPosition(target[k]);
            // Vector2d turnPos =   FieldDimensions.getTargetTurnPosition(target[k]);
            // Vector2d visionPos = FieldDimensions.getTargetVisionPosition(target[k]);
            // Vector2d hatchPos =  FieldDimensions.getTargetHatchPosition(target[k]);

            Path firstTargetPathF = new Path();
            firstTargetPathF.add(new Waypoint(startPosition, medOptions));
            firstTargetPathF.add(new Waypoint(FieldDimensions.getCargoFrontTurnPosition(), medOptions));
            firstTargetPathF.add(new Waypoint(FieldDimensions.getCargoFrontVisionPosition(), visionOptions));
            firstTargetPathF.add(new Waypoint(FieldDimensions.getCargoFrontHatchPosition(), visionOptions));
            firstTargetPathF.setReverseDirection();

        Vector2d target2StartPos =      humanStationHatchPos;
        Vector2d target2BackupTurnPos = FieldDimensions.getTargetBackupTurnPosition(target2);
        Vector2d target2TurnPos =       FieldDimensions.getTargetTurnPosition(target2);
        Vector2d target2VisionPos =     FieldDimensions.getTargetVisionPosition(target2);
        Vector2d target2HatchPos =      FieldDimensions.getTargetHatchPosition(target2);
        Vector2d target2BackupPos1 =    FieldDimensions.getTargetBackupPosition1(target2);
        Vector2d target2BackupPos2 =    FieldDimensions.getTargetBackupPosition2(target2);
        Vector2d target2BackupPos3 =    FieldDimensions.getTargetBackupPosition3(target2);

        Path secondTargetPathB1 = new Path();
        secondTargetPathB1.add(new Waypoint(humanStationHatchPos, fastOptions));
        if(target2 == FieldDimensions.TargetPositionEnum.ROCKET_FAR){
            secondTargetPathB1.add(new Waypoint(FieldDimensions.getHumanStationFarRocketMidPosition(), fastOptions));
        }
        if (target2 == FieldDimensions.TargetPositionEnum.CARGO_SIDE1 ||
            target2 == FieldDimensions.TargetPositionEnum.CARGO_SIDE2 || 
            target2 == FieldDimensions.TargetPositionEnum.CARGO_SIDE3 );
        {
            secondTargetPathB1.add(new Waypoint(FieldDimensions.getHumanStationSideCargoMidPosition(), fastOptions));
        }
        if(target2 == FieldDimensions.TargetPositionEnum.CARGO_FRONT){
            secondTargetPathB1.add(new Waypoint(FieldDimensions.getHumanStationFrontCargoMidPosition(), fastOptions));
        }
        secondTargetPathB1.add(new Waypoint(target2BackupTurnPos, fastOptions));

        Path secondTargetPathF = new Path();
        secondTargetPathF.add(new Waypoint(target2BackupTurnPos, medOptions));
        secondTargetPathF.add(new Waypoint(target2TurnPos,   medOptions));
        secondTargetPathF.add(new Waypoint(target2VisionPos, visionOptions));
        secondTargetPathF.add(new Waypoint(target2HatchPos,  visionOptions));
        secondTargetPathF.setReverseDirection();

        Path secondTargetPathB2 = new Path();
        secondTargetPathB2.add(new Waypoint(target2HatchPos,   fastOptions));
        secondTargetPathB2.add(new Waypoint(target2BackupPos1, fastOptions));
        secondTargetPathB2.add(new Waypoint(target2BackupPos2, fastOptions));
        secondTargetPathB2.add(new Waypoint(target2BackupPos3, fastOptions));



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
        runAction(new PathFollowerAction(humanStationPathF));
        runAction(new WaitAction(0.25));
        setRobotPositionAtHumanStation();


        // At Human Station: Backup to Target 2
        runAction(new PathFollowerAction(secondTargetPathB1));
        runAction(new PathFollowerAction(secondTargetPathF));
        // At Target 2: Save position, Place Hatch, then backup from target
        runAction(new HatchEjectAction());
        // backup and retract pistons
         retractDelay = 0.5;
         waitAndRetractAction = new SeriesAction(Arrays.asList(new WaitAction(retractDelay), new HatchResetAction()));
        runAction(new ParallelAction(Arrays.asList(new PathFollowerAction(secondTargetPathB2), waitAndRetractAction)));   // reverse away from target
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

    void setRobotPositionAtHumanStation()
    {
        RobotState robotState = RobotState.getInstance();
        DriveState driveState = DriveState.getInstance();
        double currentTime = Timer.getFPGATimestamp();
        
        Vector2d currentPosition = FieldDimensions.getRobotPositionAtHumanStation();
        double currentHeading = driveState.getHeading();

        robotState.setFieldToVehicle(currentTime, new Pose(currentPosition, currentHeading));
    }

}
}
