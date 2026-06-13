package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class SampleAutoPathing extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        // Start Position_End Position
        // Drive > Movement state
        // Shoot > Attempt to score the artifact

        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOTPOS_ENDPOS
    }

    PathState pathState;

    private final Pose startPose = new Pose(19.978339350180498, 120.10529482551144, Math.toRadians(142));
    private final Pose shootPose = new Pose (48.414560770156434, 92.86101083032493, Math.toRadians(142));
    private final Pose endPose = new Pose(58.20100003527967, 109.90756906923568, Math.toRadians(90));

    private PathChain driveStartToShootPos, driveShootPosEndPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
   }

   public  void statePathUpdate() {
        switch(pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartToShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD); // reset the timer and make new state
                break;
            case SHOOT_PRELOAD:
                // check if follower has completed its path?
                // and check that 5 seconds have elapsed
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    // ToDo add logic to flywheel shooter
                    follower.followPath(driveStartToShootPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS );
                    telemetry.addLine("Done Path 1");
                }
                break;
            case DRIVE_SHOOTPOS_ENDPOS:
                // All Done
                if (!follower.isBusy()) {
                    telemetry.addLine("Done All Paths");
                }
            default:
                telemetry.addLine("No State Commanded");
                break;
        }
   }

   public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
   }

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // ToDo add in any other init mechanisms

        buildPaths();
        follower.setPose(startPose);

    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("path sates", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
    }
}
