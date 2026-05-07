package org.firstinspires.ftc.teamcode.OpMode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTest extends OpMode {

    private DcMotor Intake;
    private DcMotor INTAKE;
    private Follower follower;

    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shooterpose = new Pose(68,19,Math.toRadians(121));
    private final Pose collectballs = new Pose(10,35,Math.toRadians(190));
    private final Pose getout = new Pose(35, 10, Math.toRadians(120));
    private final Pose controlpoint = new Pose(84,40.5);

    private PathChain SpSh, ShCb, CbSh, ShGo;

    public void buildPaths() {

        SpSh = follower.pathBuilder()
                .addPaths(new BezierLine(startPose, shooterpose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shooterpose.getHeading())
                .build();

        ShCb = follower.pathBuilder()
                .addPaths(new BezierCurve(shooterpose, controlpoint, collectballs))
                .setLinearHeadingInterpolation(shooterpose.getHeading(), collectballs.getHeading()) // FIX
                .build();

        CbSh = follower.pathBuilder()
                .addPaths(new BezierLine(collectballs, shooterpose))
                .setLinearHeadingInterpolation(collectballs.getHeading(), shooterpose.getHeading()) // FIX
                .build();

        ShGo = follower.pathBuilder()
                .addPaths(new BezierLine(shooterpose, getout))
                .setLinearHeadingInterpolation(shooterpose.getHeading(), getout.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                if (pathTimer.getElapsedTime() < 0.1) {
                    follower.followPath(SpSh, 1, true);
                }
                if (!follower.isBusy()) {
                    setPathState(1);
                }
                break;

            case 1:
                if (pathTimer.getElapsedTime() < 0.1) {
                    follower.followPath(ShCb, 1, true);
                }
                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                if (pathTimer.getElapsedTime() < 0.1) {
                    follower.followPath(CbSh, 1, true);
                }
                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;

            case 3:
                if (pathTimer.getElapsedTime() < 0.1) {
                    follower.followPath(ShGo, 1, true);
                }
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        INTAKE = hardwareMap.dcMotor.get("INTAKE");
        Intake = hardwareMap.dcMotor.get("Intake");
        pathState = 0;

    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
        telemetry.addData("isBusy", follower.isBusy());
        telemetry.addData("timer", pathTimer.getElapsedTime());
    }

    @Override
    public void stop() {}
}