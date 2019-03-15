package frc.robot.auto.modes;

import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.SmartDashboardInteractions;
import frc.robot.auto.AutoModeBase;
import frc.robot.auto.AutoModeEndedException;
import frc.robot.auto.actions.Action;
import frc.robot.auto.actions.HatchCollisionDetectionAction;
import frc.robot.auto.actions.HatchEjectAction;
import frc.robot.auto.actions.HatchResetAction;
import frc.robot.auto.actions.InterruptableAction;
import frc.robot.auto.actions.ParallelAction;
import frc.robot.auto.actions.PathFollowerAction;
import frc.robot.auto.actions.SeriesAction;
import frc.robot.auto.actions.WaitAction;
import frc.robot.command_status.DriveState;
import frc.robot.command_status.RobotState;
import frc.robot.lib.sensors.Limelight;
import frc.robot.lib.util.Path;
import frc.robot.lib.util.Path.Waypoint;
import frc.robot.lib.util.PathSegment;
import frc.robot.lib.util.Pose;
import frc.robot.lib.util.Vector2d;

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

        double   fastSpeed = 72;
        double    medSpeed = 48;
        double   slowSpeed = 24;
        double visionSpeed = 36;

        double     accelTime = 0.5;     // time to accelerate to full speed
        double lookaheadTime = 1.0;     // time to lookahead

        double visionLookaheadDist = 24;

        PathSegment.Options   fastOptions = new PathSegment.Options(  fastSpeed,   fastSpeed/accelTime, fastSpeed/lookaheadTime, false);
        PathSegment.Options    medOptions = new PathSegment.Options(   medSpeed,    medSpeed/accelTime,  medSpeed/lookaheadTime, false);
        PathSegment.Options   slowOptions = new PathSegment.Options(  slowSpeed,   slowSpeed/accelTime, slowSpeed/lookaheadTime, false);
        PathSegment.Options visionOptions = new PathSegment.Options(visionSpeed, visionSpeed/accelTime,     visionLookaheadDist, true);

        double retractDelay = 0.5;

        SmartDashboardInteractions smartDashboardInteractions = SmartDashboardInteractions.getInstance();
        Pose startPose = smartDashboardInteractions.getStartPosition();
        Vector2d startPosition = startPose.getPosition();

        double startDelaySec = smartDashboardInteractions.getStartDelay();

        FieldDimensions.TargetPositionEnum target1 = smartDashboardInteractions.getAutoFirstTarget();
        FieldDimensions.TargetPositionEnum target2 = smartDashboardInteractions.getAutoSecondTarget();

        //============================================================================
        // Target 1
        //============================================================================

        Vector2d target1StartPos =      startPosition;
        Vector2d target1BackupTurnPos = FieldDimensions.getTargetBackupTurnPosition(target1);
        Vector2d target1TurnPos =       FieldDimensions.getTargetTurnPosition(target1);
        Vector2d target1VisionPos1 =    FieldDimensions.getTargetVisionPosition1(target1);
        Vector2d target1VisionPos2 =    FieldDimensions.getTargetVisionPosition2(target1);
        Vector2d target1HatchPos =      FieldDimensions.getTargetHatchPosition(target1);
        Vector2d target1BackupPos1 =    FieldDimensions.getTargetBackupPosition1(target1);
        Vector2d target1BackupPos2 =    FieldDimensions.getTargetBackupPosition2(target1);
        Vector2d target1BackupPos3 =    FieldDimensions.getTargetBackupPosition3(target1);

        Path firstTargetPathB1;
        Path firstTargetPathF;
        if (target1 == FieldDimensions.TargetPositionEnum.CARGO_FRONT)
        {
            firstTargetPathB1 = new Path();    // no backup

            firstTargetPathF = new Path();
            firstTargetPathF.add(new Waypoint(target1StartPos,   medOptions));       // drive slowly off of hab
            firstTargetPathF.add(new Waypoint(target1TurnPos,    medOptions));
            firstTargetPathF.add(new Waypoint(target1VisionPos1, medOptions));
            firstTargetPathF.setReverseDirection();
        }
        else    // side cargo
        {
            firstTargetPathB1 = new Path();
            firstTargetPathB1.add(new Waypoint(target1StartPos,      fastOptions));
            firstTargetPathB1.add(new Waypoint(target1BackupTurnPos, fastOptions));
    
            firstTargetPathF = new Path();
            firstTargetPathF.add(new Waypoint(target1BackupTurnPos,     medOptions));       // drive slowly off of hab
            firstTargetPathF.add(new Waypoint(target1TurnPos,           medOptions));
            firstTargetPathF.add(new Waypoint(target1VisionPos1,      visionOptions));    // turn on leds, use vision
            firstTargetPathF.setReverseDirection();
        }

        Path firstTargetPathV = new Path();
        firstTargetPathV.add(new Waypoint(target1VisionPos2, visionOptions));    // turn on leds, use vision
        firstTargetPathV.add(new Waypoint(target1HatchPos,  visionOptions));    // target hatch
        firstTargetPathV.setReverseDirection();

        Path firstTargetPathB = new Path();
        firstTargetPathB.add(new Waypoint(target1HatchPos,   fastOptions));     // backup quickly
        firstTargetPathB.add(new Waypoint(target1BackupPos1, fastOptions));
        firstTargetPathB.add(new Waypoint(target1BackupPos2, fastOptions));
        firstTargetPathB.add(new Waypoint(target1BackupPos3, fastOptions));

        //============================================================================
        // Human Station
        //============================================================================

        Vector2d humanStationTurnPos =   FieldDimensions.getHumanStationTurnPosition();
        Vector2d humanStationVisionPos1 = FieldDimensions.getHumanStationVisionPosition1();
        Vector2d humanStationVisionPos2 = FieldDimensions.getHumanStationVisionPosition2();
        Vector2d humanStationHatchPos =  FieldDimensions.getHumanStationHatchPosition();
        
        Path humanStationPathF = new Path();
        humanStationPathF.add(new Waypoint(target1BackupPos3, fastOptions));
        if(target1 == FieldDimensions.TargetPositionEnum.ROCKET_FAR)
        {
            humanStationPathF.add(new Waypoint(FieldDimensions.getHumanStationFarRocketMidPosition(), fastOptions));
        }
        if (target1 == FieldDimensions.TargetPositionEnum.CARGO_SIDE1 ||
            target1 == FieldDimensions.TargetPositionEnum.CARGO_SIDE2 || 
            target1 == FieldDimensions.TargetPositionEnum.CARGO_SIDE3 );
        {
            humanStationPathF.add(new Waypoint(FieldDimensions.getHumanStationSideCargoMidPosition(), fastOptions));
        }
        humanStationPathF.add(new Waypoint(humanStationTurnPos,    medOptions));  // slow down a bit to avoid jerkiness
        humanStationPathF.add(new Waypoint(humanStationVisionPos1, medOptions));
        humanStationPathF.setReverseDirection();

        Path humanStationPathV = new Path();
        humanStationPathV.add(new Waypoint(humanStationVisionPos2, visionOptions));
        humanStationPathV.add(new Waypoint(humanStationHatchPos,   visionOptions));
        humanStationPathV.setReverseDirection();

  





        //============================================================================
        // Actions
        //============================================================================

        // setup LEDs
        Limelight.getCargoInstance().setLEDMode(Limelight.LedMode.kOff);
        Limelight.getHatchInstance().setLEDMode(Limelight.LedMode.kOn);

        // optional start delay
        runAction(new WaitAction(startDelaySec));               // initial delay (optional)
        
        // At Starting Position: Go to Target 1
        if (target1 != FieldDimensions.TargetPositionEnum.CARGO_FRONT)
        {
            runAction(new PathFollowerAction(firstTargetPathB1));
        }
        runAction(new PathFollowerAction(firstTargetPathF));    // drive off platform towards first target
        runAction(new InterruptableAction(new HatchCollisionDetectionAction(), new PathFollowerAction(firstTargetPathV)));    // score

        // At Target 1:  Save position, Place Hatch, then backup from target
        // setRobotPosition(target1);
        runAction(new HatchEjectAction()); //eject hatch action
        Action waitAndRetractAction = new SeriesAction(Arrays.asList(new WaitAction(retractDelay), new HatchResetAction()));
        runAction(new ParallelAction(Arrays.asList(new PathFollowerAction(firstTargetPathB), waitAndRetractAction)));   // reverse away from target


        // Backed up from Target 1: Drive to Human Station
        runAction(new PathFollowerAction(humanStationPathF));
        runAction(new InterruptableAction(new HatchCollisionDetectionAction(), new PathFollowerAction(humanStationPathV)));
        runAction(new WaitAction(0.25));
        // setRobot PositionAtHumanStation();

        // Done!

    }

    void setRobotPosition(FieldDimensions.TargetPositionEnum _target)
    {
        RobotState robotState = RobotState.getInstance();
        DriveState driveState = DriveState.getInstance();
        double currentTime = Timer.getFPGATimestamp();
        
        Vector2d currentPosition = FieldDimensions.getRobotPositionAtTarget(_target);
        double currentHeading = driveState.getHeading();

        robotState.reset(new Pose(currentPosition, currentHeading));
    }

    void setRobotPositionAtHumanStation()
    {
        RobotState robotState = RobotState.getInstance();
        DriveState driveState = DriveState.getInstance();
        double currentTime = Timer.getFPGATimestamp();
        
        Vector2d currentPosition = FieldDimensions.getRobotPositionAtHumanStation();
        double currentHeading = driveState.getHeading();

        robotState.reset(new Pose(currentPosition, currentHeading));
    }

}
