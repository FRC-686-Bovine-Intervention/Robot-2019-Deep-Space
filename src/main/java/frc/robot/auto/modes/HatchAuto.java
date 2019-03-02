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
        double speed = 36;    //DriveLoop.kPathFollowingMaxVel;
        double visionSpeed = 36;    // slow down for collision/score
        double accel = 24;  // DriveLoop.kPathFollowingMaxAccel
        double lookaheadDist = DriveLoop.kPathFollowingLookahead;


        PathSegment.Options pathOptions	=       new PathSegment.Options(speed, accel, lookaheadDist, false);
        PathSegment.Options tightTurnOptions =  new PathSegment.Options(speed, accel, lookaheadDist/2, false);
        PathSegment.Options visionOptions =     new PathSegment.Options(visionSpeed, accel, lookaheadDist, true);

        SmartDashboardInteractions smartDashboardInteractions = SmartDashboardInteractions.getInstance();
        Pose startPose = smartDashboardInteractions.getStartPosition();
        Vector2d startPosition = startPose.getPosition();

        double startDelaySec = smartDashboardInteractions.getStartDelay();

        TargetPositionEnum target[] = new TargetPositionEnum[2];
        target[0] = smartDashboardInteractions.getAutoFirstTarget();
        target[1] = smartDashboardInteractions.getAutoSecondTarget();

        Vector2d humanStationVisionPos = FieldDimensions.getHumanStationVisionPosition();
        Vector2d humanStationHatchPos =  FieldDimensions.getHumanStationHatchPosition();

        for (int k = 0; k < 2; k++)
        {
            Vector2d startPos = new Vector2d();
            if (k==0) {
                startPos = startPosition;           // 1st target: start from HAB
            }
            else {
                startPos = humanStationHatchPos;    // 2nd target: start from human station
            }

            Vector2d backupPos = FieldDimensions.getTargetBackupPosition(target[k]);
            Vector2d turnPos =   FieldDimensions.getTargetTurnPosition(target[k]);
            Vector2d visionPos = FieldDimensions.getTargetVisionPosition(target[k]);
            Vector2d hatchPos =  FieldDimensions.getTargetHatchPosition(target[k]);

            Path startToBackup = new Path();
            startToBackup.add(new Waypoint(startPos,    pathOptions));
            startToBackup.add(new Waypoint(backupPos,   pathOptions));
            
            Path backupToScore = new Path();
            backupToScore.add(new Waypoint(backupPos,   pathOptions));
            backupToScore.add(new Waypoint(turnPos,     pathOptions));
            backupToScore.add(new Waypoint(visionPos,   pathOptions));
            backupToScore.add(new Waypoint(hatchPos,    visionOptions));    // use vision after turning towards target
            backupToScore.setReverseDirection();

            Path scoreToBackup = new Path();
            scoreToBackup.add(new Waypoint(hatchPos,    pathOptions));
            scoreToBackup.add(new Waypoint(backupPos,   pathOptions));

            Path backupToHumanStation = new Path();
            backupToHumanStation.add(new Waypoint(backupPos,               pathOptions));
            backupToHumanStation.add(new Waypoint(humanStationVisionPos,   pathOptions));
            backupToHumanStation.add(new Waypoint(humanStationHatchPos,    visionOptions)); // use vision after turning towards target
            backupToHumanStation.setReverseDirection();


            if (k==0) {
                runAction(new WaitAction(startDelaySec));
            }

            runAction(new PathFollowerAction(startToBackup));   // drive off platform 
            runAction(new PathFollowerAction(backupToScore));   // to target

            runAction(new HatchEjectAction()); //eject hatch action

            // backup and retract pistons
            double retractDelay = 0.5;
            Action waitAndRetractAction = new SeriesAction(Arrays.asList(new WaitAction(retractDelay), new HatchResetAction()));
            runAction(new ParallelAction(Arrays.asList(new PathFollowerAction(scoreToBackup), waitAndRetractAction)));   // reverse away from target
    
            if (k==0) {
                runAction(new PathFollowerAction(backupToHumanStation));    // to human station
            }
    }
}
}
