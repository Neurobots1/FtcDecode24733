package org.firstinspires.ftc.teamcode.OpMode; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutoTest extends OpMode {
    public ShooterSubsytem shooter;//fais appelle au Code ShooterSubsystem

    private DcMotor Intake;
    private DcMotor INTAKE;
    private Follower follower;
    double currentTPS = getCurrentTPS();

    private double getCurrentTPS() {
        return 0;
    }

    private Timer pathTimer, opmodeTimer;
    private int pathState;

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90));
    private final Pose shooterpose = new Pose(68,19,Math.toRadians(121));
    private final Pose collectballs = new Pose(10,35,Math.toRadians(190));
    private final Pose getout = new Pose(35, 10, Math.toRadians(120));
    private final Pose controlpoint = new Pose(84,40.5);
    public double TARGET_TPS = 1360;



    private PathChain s,SpSh,ShCb,CbSh,ShGo;
    public void buildPaths() {

        SpSh = follower.pathBuilder()
                .addPaths(new BezierLine(startPose,shooterpose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shooterpose.getHeading())
                .build();
        ShCb = follower.pathBuilder()
                .addPaths(new BezierCurve(shooterpose,controlpoint,collectballs))
                .setLinearHeadingInterpolation(collectballs.getHeading(),shooterpose.getHeading())
                .build();
        CbSh =follower.pathBuilder()
                .addPaths(new BezierLine(collectballs, shooterpose))
                .setLinearHeadingInterpolation(shooterpose.getHeading(),getout .getHeading())
                .build();


        ShGo =follower.pathBuilder()
                .addPaths(new BezierLine(shooterpose, getout))
                .setLinearHeadingInterpolation(shooterpose.getHeading(),getout .getHeading())
                .build();




    }
    public void shoot (){

        TARGET_TPS = 1540;
        if ((TARGET_TPS-getCurrentTPS()>10))  {
            Intake.setPower(1);
            INTAKE.setPower(1);
        }

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(SpSh,1,true);
                if (!follower.isBusy()){
                    setPathState(1);
                }
                break;


            case 1:
                follower.followPath(ShCb,1,true);

                if (!follower.isBusy()){
                    setPathState(2);
                }
                break;

            case 2:
                follower.followPath(CbSh,1,true);
                if (!follower.isBusy()){
                    setPathState(3);
                }
                break;
            case 3:
                follower.followPath(ShGo,1,true);
                if (!follower.isBusy()){
                    setPathState(-1);
                }
                break;


        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }



    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        INTAKE = hardwareMap.dcMotor.get("INTAKE");
        Intake = hardwareMap.dcMotor.get("Intake");

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
    }

    @Override
    public void stop() {}
}

