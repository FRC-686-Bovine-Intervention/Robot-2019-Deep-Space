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

        Vector2d humanStationVisionPos = FieldDimensions.getHumanStationVisionPosition();
        Vector2d humanStationHatchPos =  FieldDimensions.getHumanStationHatchPosition();

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

            Path firstTargetPathB = new Path();
            firstTargetPathB.add(new Waypoint(FieldDimensions.getCargoFrontHatchPosition(), fastOptions));
            firstTargetPathB.add(new Waypoint(FieldDimensions.getCargoFrontBackupPosition1(), fastOptions));
            firstTargetPathB.add(new Waypoint(FieldDimensions.getCargoFrontBackupPosition2(), fastOptions));
            firstTargetPathB.add(new Waypoint(FieldDimensions.getCargoFrontBackupPosition3(), fastOptions));


            Limelight.getCargoInstance().setLEDMode(Limelight.LedMode.kOff);
            Limelight.getHatchInstance().setLEDMode(Limelight.LedMode.kOn);
    

            
            runAction(new WaitAction(startDelaySec));
            runAction(new PathFollowerAction(firstTargetPathF));   // drive off platform 
            //setRobotPosition(targetHatch);
            runAction(new HatchEjectAction()); //eject hatch action
            // backup and retract pistons
            double retractDelay = 0.5;
            Action waitAndRetractAction = new SeriesAction(Arrays.asList(new WaitAction(retractDelay), new HatchResetAction()));
            runAction(new ParallelAction(Arrays.asList(new PathFollowerAction(firstTargetPathB), waitAndRetractAction)));   // reverse away from target
            // runAction(new PathFollowerAction(backupToHumanStation));    // to human station
            
    
}
}
