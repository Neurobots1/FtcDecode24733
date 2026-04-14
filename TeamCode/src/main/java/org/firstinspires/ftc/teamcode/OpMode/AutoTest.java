package org.firstinspires.ftc.teamcode.OpMode; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "AutoTest", group = "Examples")
public class AutoTest extends OpMode {

    private Follower follower;

    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(72, 72, Math.toRadians(180));
    private final Pose ballePose1= new Pose(72,50,Math.toRadians(45));




    private PathChain s,test;
    public void buildPaths() {

        test = follower.pathBuilder()
                .addPaths(new BezierLine(startPose,ballePose1))
                .setLinearHeadingInterpolation(startPose.getHeading(),ballePose1.getHeading())
                .build();

    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(test,1,true);
                if (!follower.isBusy()){
                    setPathState(-1);
                }
                break;


            case -1:
                break;

        }
    }

    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


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
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {}
}

