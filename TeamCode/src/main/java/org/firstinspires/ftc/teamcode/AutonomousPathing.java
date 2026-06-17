package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@SuppressWarnings({"FieldMayBeFinal","FieldCanBeLocal"})
//@Disabled
@Autonomous
public class AutonomousPathing  extends OpMode {

    // ── Alliance Selection ──────────────────────────────────────────────────
    public enum Alliance { RED, BLUE }
    private Alliance alliance = Alliance.RED; // default

    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private int rowNumber = 0;

    public enum PathState {
        DRIVE_START_POS_SHOOT_POS,
        SHOOT_PRELOAD,
        DRIVE_SHOOT_POS_FIRST_ROW_START_POS,
        DRIVE_FIRST_ROW_START_POS_FIRST_ROW_COLLECT_POS,
        DRIVE_SHOOT_POS_SECOND_ROW_START_POS,
        DRIVE_SECOND_ROW_START_POS_SECOND_ROW_COLLECT_POS,
        DRIVE_SHOOT_POS_THIRD_ROW_START_POS,
        DRIVE_THIRD_ROW_START_POS_THIRD_ROW_COLLECT_POS,
        DRIVE_SHOOT_POS_END_POS
    }

    PathState pathState;

    // ── Red Alliance Poses ──────────────────────────────────────────────────
    private final Pose redStartPose           = new Pose(107.93029871977241, 132.99502133712662, Math.toRadians(90));
    private final Pose redShootPose           = new Pose(95.6522048364154,   95.37766714082504,  Math.toRadians(49));
    private final Pose redFirstRowStartPose   = new Pose(95.6522048364154,   95.37766714082504,  Math.toRadians(49));
    private final Pose redFirstRowCollectPose = new Pose(95.6522048364154,   82.44641989554287,  Math.toRadians(0));
    private final Pose redSecondRowStartPose  = new Pose(95.6522048364154,   58.06491511789809,  Math.toRadians(0));
    private final Pose redSecondRowCollectPose= new Pose(122.93023683875481, 58.416446387850236, Math.toRadians(0));
    private final Pose redThirdRowStartPose   = new Pose(95.6522048364154,   34.97892958923724,  Math.toRadians(0));
    private final Pose redThirdRowCollectPose = new Pose(122.93023683875481, 34.92789814599325,  Math.toRadians(0));
    private final Pose redEndPose             = new Pose(82.18655975729182,  121.65240589995616, Math.toRadians(270));

    // ── Blue Alliance Poses (mirrored across field Y-axis, 144" wide field) ─
    private final Pose blueStartPose           = new Pose(144 - 107.93029871977241, 132.99502133712662, Math.toRadians(90));
    private final Pose blueShootPose           = new Pose(144 - 95.6522048364154,   95.37766714082504,  Math.toRadians(131));
    private final Pose blueFirstRowStartPose   = new Pose(144 - 95.6522048364154,   95.37766714082504,  Math.toRadians(131));
    private final Pose blueFirstRowCollectPose = new Pose(144 - 95.6522048364154,   82.44641989554287,  Math.toRadians(180));
    private final Pose blueSecondRowStartPose  = new Pose(144 - 95.6522048364154,   58.06491511789809,  Math.toRadians(180));
    private final Pose blueSecondRowCollectPose= new Pose(144 - 122.93023683875481, 58.416446387850236, Math.toRadians(180));
    private final Pose blueThirdRowStartPose   = new Pose(144 - 95.6522048364154,   34.97892958923724,  Math.toRadians(180));
    private final Pose blueThirdRowCollectPose = new Pose(144 - 122.93023683875481, 34.92789814599325,  Math.toRadians(180));
    private final Pose blueEndPose             = new Pose(144 - 82.18655975729182,  121.65240589995616, Math.toRadians(270));

    // ── Active Poses (set in start() based on alliance selection) ───────────
    private Pose startPose, shootPose, firstRowStartPose, firstRowCollectPose;
    private Pose secondRowStartPose, secondRowCollectPose;
    private Pose thirdRowStartPose, thirdRowCollectPose, endPose;

    private PathChain
            driveStartPosToShootPos,
            driveShootPosToFirstRowStartPos,
            driveFirstRowStartPosToFirstRowCollectPos,
            driveFirstRowCollectPosToShootPos,
            driveShootPosToSecondRowStartPos,
            driveSecondRowStartPosToSecondRowCollectPos,
            driveSecondRowCollectPosToShootPos,
            driveShootPosToThirdRowStartPos,
            driveThirdRowStartPosToThirdRowCollectPos,
            driveThirdRowCollectPosToShootPos,
            driveShootPosToEndPose;

    // ── Apply the selected alliance's poses to the active pose variables ────
    private void applyAlliancePoses() {
        if (alliance == Alliance.RED) {
            startPose            = redStartPose;
            shootPose            = redShootPose;
            firstRowStartPose    = redFirstRowStartPose;
            firstRowCollectPose  = redFirstRowCollectPose;
            secondRowStartPose   = redSecondRowStartPose;
            secondRowCollectPose = redSecondRowCollectPose;
            thirdRowStartPose    = redThirdRowStartPose;
            thirdRowCollectPose  = redThirdRowCollectPose;
            endPose              = redEndPose;
        } else {
            startPose            = blueStartPose;
            shootPose            = blueShootPose;
            firstRowStartPose    = blueFirstRowStartPose;
            firstRowCollectPose  = blueFirstRowCollectPose;
            secondRowStartPose   = blueSecondRowStartPose;
            secondRowCollectPose = blueSecondRowCollectPose;
            thirdRowStartPose    = blueThirdRowStartPose;
            thirdRowCollectPose  = blueThirdRowCollectPose;
            endPose              = blueEndPose;
        }
    }

    public void buildPaths() {
        driveStartPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosToFirstRowStartPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, firstRowStartPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), firstRowStartPose.getHeading())
                .build();
        driveFirstRowStartPosToFirstRowCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(firstRowStartPose, firstRowCollectPose))
                .setLinearHeadingInterpolation(firstRowStartPose.getHeading(), firstRowCollectPose.getHeading())
                .build();
        driveFirstRowCollectPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(firstRowCollectPose, shootPose))
                .setLinearHeadingInterpolation(firstRowCollectPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosToSecondRowStartPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, secondRowStartPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), secondRowStartPose.getHeading())
                .build();
        driveSecondRowStartPosToSecondRowCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(secondRowStartPose, secondRowCollectPose))
                .setLinearHeadingInterpolation(secondRowStartPose.getHeading(), secondRowCollectPose.getHeading())
                .build();
        driveSecondRowCollectPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(secondRowCollectPose, shootPose))
                .setLinearHeadingInterpolation(secondRowCollectPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosToThirdRowStartPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, thirdRowStartPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), thirdRowStartPose.getHeading())
                .build();
        driveThirdRowStartPosToThirdRowCollectPos = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowStartPose, thirdRowCollectPose))
                .setLinearHeadingInterpolation(thirdRowStartPose.getHeading(), thirdRowCollectPose.getHeading())
                .build();
        driveThirdRowCollectPosToShootPos = follower.pathBuilder()
                .addPath(new BezierLine(thirdRowCollectPose, shootPose))
                .setLinearHeadingInterpolation(thirdRowCollectPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosToEndPose = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();
    }

    public void statePathUpdate() {
        switch(pathState) {
            case DRIVE_START_POS_SHOOT_POS:
                follower.followPath(driveStartPosToShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;
            case SHOOT_PRELOAD:
                if(!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    rowNumber++;
                    switch (rowNumber) {
                        case 1:
                            follower.followPath(driveShootPosToFirstRowStartPos, true);
                            setPathState(PathState.DRIVE_SHOOT_POS_FIRST_ROW_START_POS);
                            telemetry.addLine("Starting Row 1");
                            break;
                        case 2:
                            follower.followPath(driveShootPosToSecondRowStartPos, true);
                            setPathState(PathState.DRIVE_SHOOT_POS_SECOND_ROW_START_POS);
                            telemetry.addLine("Starting Row 2");
                            break;
                        case 3:
                            follower.followPath(driveShootPosToThirdRowStartPos, true);
                            setPathState(PathState.DRIVE_SHOOT_POS_THIRD_ROW_START_POS);
                            telemetry.addLine("Starting Row 3");
                            break;
                        default:
                            follower.followPath(driveShootPosToEndPose, true);
                            setPathState(PathState.DRIVE_SHOOT_POS_END_POS);
                            telemetry.addLine("All Artifacts Fired");
                            break;
                    }
                }
                break;
            case DRIVE_SHOOT_POS_FIRST_ROW_START_POS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveFirstRowStartPosToFirstRowCollectPos, true);
                    setPathState(PathState.DRIVE_FIRST_ROW_START_POS_FIRST_ROW_COLLECT_POS);
                }
                break;
            case DRIVE_FIRST_ROW_START_POS_FIRST_ROW_COLLECT_POS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveFirstRowCollectPosToShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case DRIVE_SHOOT_POS_SECOND_ROW_START_POS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveSecondRowStartPosToSecondRowCollectPos, true);
                    setPathState(PathState.DRIVE_SECOND_ROW_START_POS_SECOND_ROW_COLLECT_POS);
                }
                break;
            case DRIVE_SECOND_ROW_START_POS_SECOND_ROW_COLLECT_POS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveSecondRowCollectPosToShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case DRIVE_SHOOT_POS_THIRD_ROW_START_POS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveThirdRowStartPosToThirdRowCollectPos, true);
                    setPathState(PathState.DRIVE_THIRD_ROW_START_POS_THIRD_ROW_COLLECT_POS);
                }
                break;
            case DRIVE_THIRD_ROW_START_POS_THIRD_ROW_COLLECT_POS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveThirdRowCollectPosToShootPos, true);
                    setPathState(PathState.SHOOT_PRELOAD);
                }
                break;
            case DRIVE_SHOOT_POS_END_POS:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    telemetry.addData("Full Path time", pathTimer.getElapsedTimeSeconds());
                    telemetry.addLine("End of Autonomous");
                }
                break;
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
        pathState = PathState.DRIVE_START_POS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        telemetry.addLine("=== Alliance Selection ===");
        telemetry.addLine("Press X (Blue) or B (Red) to select alliance");
        telemetry.addData("Current Alliance", alliance.toString());
        telemetry.update();
    }

    // ── Runs repeatedly during init so the driver can toggle alliance ────────
    @Override
    public void init_loop() {
        if (gamepad1.x) {
            alliance = Alliance.BLUE;
        } else if (gamepad1.b) {
            alliance = Alliance.RED;
        }

        telemetry.addLine("=== Alliance Selection ===");
        telemetry.addLine("Press X (Blue) or B (Red)");
        telemetry.addData("Selected Alliance", alliance == Alliance.RED ? "🔴 RED" : "🔵 BLUE");
        telemetry.addLine("Press Play when ready.");
        telemetry.update();
    }

    @Override
    public void start() {
        applyAlliancePoses();       // lock in the chosen pose set
        buildPaths();               // build paths from the active poses
        follower.setPose(startPose);

        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();

        telemetry.addData("Alliance",   alliance.toString());
        telemetry.addData("Path State", pathState.toString());
        telemetry.addData("x",          follower.getPose().getX());
        telemetry.addData("y",          follower.getPose().getY());
        telemetry.addData("heading",    follower.getPose().getHeading());
        telemetry.addData("Path time",  pathTimer.getElapsedTimeSeconds());
    }
}