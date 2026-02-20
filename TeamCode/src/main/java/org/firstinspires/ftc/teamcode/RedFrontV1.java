package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Front ", group = "Examples")

public class RedFrontV1 extends OpMode {

    // ---------------- HARDWARE ----------------
    // ---------------- PEDRO ----------------
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    // posepose
    // posepose
    private final Pose startPose = new Pose(84, 18, Math.toRadians(90));
    private final Pose scorePose = new Pose(84, 25, Math.toRadians(65));
    private final Pose toArtifactLine1 = new Pose(100, 42.5, Math.toRadians(0));
    private final Pose driveThroughLine1 = new Pose(136, 42.5, Math.toRadians(0));
    private final Pose driveToShoot2 = new Pose(83, 25, Math.toRadians(62.5));
    private final Pose toArtifactLine2 = new Pose(100, 66.5, Math.toRadians(0));
    private final Pose driveThroughLine2 = new Pose(136, 66.5, Math.toRadians(0));
    private final Pose driveToShoot3 = new Pose(83, 25, Math.toRadians(64));
    private final Pose leavePose = new Pose(100, 36, Math.toRadians(180));

    private Path scorePreload;
    private PathChain driveToLine1, pickUpLine1, goShoot2, driveToLine2, pickUpLine2, goShoot3, leaveOutChain;
    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        driveToLine1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, toArtifactLine1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), toArtifactLine1.getHeading())
                .build();

        pickUpLine1 = follower.pathBuilder()
                .addPath(new BezierLine(toArtifactLine1, driveThroughLine1))
                .setLinearHeadingInterpolation(toArtifactLine1.getHeading(), driveThroughLine1.getHeading())
                .build();

        goShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(driveThroughLine1, driveToShoot2))
                .setLinearHeadingInterpolation(driveThroughLine1.getHeading(), driveToShoot2.getHeading())
                .build();

        driveToLine2 = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot2, toArtifactLine2))
                .setLinearHeadingInterpolation(driveToShoot2.getHeading(), toArtifactLine2.getHeading())
                .build();

        pickUpLine2 = follower.pathBuilder()
                .addPath(new BezierLine(toArtifactLine2, driveThroughLine2))
                .setLinearHeadingInterpolation(toArtifactLine2.getHeading(), driveThroughLine2.getHeading())
                .build();

        goShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(driveThroughLine2, driveToShoot3))
                .setLinearHeadingInterpolation(driveThroughLine2.getHeading(), driveToShoot3.getHeading())
                .build();

        leaveOutChain = follower.pathBuilder()
                .addPath(new BezierLine(driveToShoot3, leavePose))
                .setLinearHeadingInterpolation(driveToShoot3.getHeading(), leavePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:

            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */

                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Preload */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(driveToLine1,true);
                    setPathState(2);
                }
                break;
            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(pickUpLine1,true);
                    setPathState(3);
                }
                break;
            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(goShoot2,true);
                    setPathState(4);
                }
                break;
            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(driveToLine2,true);
                    setPathState(5);
                }
                break;
            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Score Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(pickUpLine2,true);
                    setPathState(6);
                }
                break;
            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup3Pose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */

                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(goShoot3, true);
                    setPathState(7);
                }
                break;
            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                    follower.getVelocity();
                }
                break;
        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    // ---------------- MAIN LOOP ----------------
    @Override
    public void loop() {



        follower.update();
        autonomousPathUpdate();




        // ---- Telemetry ----
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    // ---------------- INIT / START ----------------
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);



        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void stop() {
        // no-op
    }


}