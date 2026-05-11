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

    public ShooterSubsytem shooter;

    private DcMotor Intake;
    private DcMotor INTAKE;

    private Follower follower;

    private Timer pathTimer;
    private Timer opmodeTimer;

    private int pathState = -1;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shooterPose = new Pose(68, 19, Math.toRadians(121));
    private final Pose collectBalls = new Pose(10, 35, Math.toRadians(190));
    private final Pose getOut = new Pose(35, 10, Math.toRadians(120));

    private final Pose controlPoint = new Pose(84, 40.5);

    public static double TARGET_TPS = 1540;
    public static double SHOOTER_TOLERANCE_TPS = 50;

    private PathChain SpSh;
    private PathChain ShCb;
    private PathChain CbSh;
    private PathChain ShGo;

    public void buildPaths() {

        SpSh = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shooterPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shooterPose.getHeading())
                .build();

        ShCb = follower.pathBuilder()
                .addPath(new BezierCurve(shooterPose, controlPoint, collectBalls))
                .setLinearHeadingInterpolation(shooterPose.getHeading(), collectBalls.getHeading())
                .build();

        CbSh = follower.pathBuilder()
                .addPath(new BezierLine(collectBalls, shooterPose))
                .setLinearHeadingInterpolation(collectBalls.getHeading(), shooterPose.getHeading())
                .build();

        ShGo = follower.pathBuilder()
                .addPath(new BezierLine(shooterPose, getOut))
                .setLinearHeadingInterpolation(shooterPose.getHeading(), getOut.getHeading())
                .build();
    }

    public void shoot() {
        shooter.start();

        double currentTPS = shooter.getCurrentTPS();
        double error = TARGET_TPS - currentTPS;

        if (Math.abs(error) <= SHOOTER_TOLERANCE_TPS) {
            Intake.setPower(1);
            INTAKE.setPower(1);
        } else {
            Intake.setPower(0);
            INTAKE.setPower(0);
        }
    }

    public void stopShooting() {
        shooter.stop();

        Intake.setPower(0);
        INTAKE.setPower(0);
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            case 0:
                shoot();

                if (!follower.isBusy()) {
                    setPathState(1);
                }
                break;

            case 1:
                stopShooting();

                if (!follower.isBusy()) {
                    setPathState(2);
                }
                break;

            case 2:
                shoot();

                if (!follower.isBusy()) {
                    setPathState(3);
                }
                break;

            case 3:
                stopShooting();

                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;

            case -1:
                stopShooting();
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();

        switch (pathState) {

            case 0:
                follower.followPath(SpSh, 1, true);
                break;

            case 1:
                follower.followPath(ShCb, 1, true);
                break;

            case 2:
                follower.followPath(CbSh, 1, true);
                break;

            case 3:
                follower.followPath(ShGo, 1, true);
                break;

            case -1:
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        buildPaths();

        shooter = new ShooterSubsytem(hardwareMap);

        INTAKE = hardwareMap.dcMotor.get("INTAKE");
        Intake = hardwareMap.dcMotor.get("Intake");

        shooter.stop();

        Intake.setPower(0);
        INTAKE.setPower(0);
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

        shooter.update();

        telemetry.addData("Path State", pathState);

        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());

        telemetry.addData("Target TPS", TARGET_TPS);
        telemetry.addData("Current TPS", shooter.getCurrentTPS());
        telemetry.addData("Shooter Error", TARGET_TPS - shooter.getCurrentTPS());
        telemetry.addData("Shooter At Speed", Math.abs(TARGET_TPS - shooter.getCurrentTPS()) <= SHOOTER_TOLERANCE_TPS);
        telemetry.addData("Shooter Power", shooter.getLastPower());

        telemetry.update();
    }

    @Override
    public void stop() {
        stopShooting();
    }
}