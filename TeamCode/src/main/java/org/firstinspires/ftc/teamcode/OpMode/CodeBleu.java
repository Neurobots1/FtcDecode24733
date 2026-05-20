package org.firstinspires.ftc.teamcode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class CodeBleu extends OpMode {

    public ShooterSubsytem shooter;
    private AutoAimShooter autoAimShooter;
    private Follower follower;

    private Servo Servorot;

    private DcMotor FrontL;
    private DcMotor FrontR;
    private DcMotor BackL;
    private DcMotor BackR;

    private DcMotor Shooter2;

    private DcMotor Intake;
    private DcMotor INTAKE;

    public static double RESET_X = 6;
    public static double RESET_Y = 6;
    public static double RESET_HEADING = 0;

    private boolean lastYPressed = false;
    private boolean lastAPressed = false;

    private boolean safeMode = false;

    public static double TARGET_TPS = 900;

    public static double GATE_OPEN = 0.69;
    public static double GATE_CLOSED = 0.0;

    public static double TRIGGER_THRESHOLD = 0.05;

    private boolean autoAimActive = false;
    private boolean isInAZone = false;

    public enum ShooterState {
        OFF,
        SPINNING_UP,
        READY
    }

    private ShooterState shooterState = ShooterState.OFF;

    @Override
    public void init() {
        shooter = new ShooterSubsytem(hardwareMap);
        autoAimShooter = new AutoAimShooter();

        Servorot = hardwareMap.get(Servo.class, "Servorot");

        INTAKE = hardwareMap.dcMotor.get("INTAKE");
        Intake = hardwareMap.dcMotor.get("Intake");

        FrontL = hardwareMap.dcMotor.get("FrontL");
        FrontR = hardwareMap.dcMotor.get("FrontR");
        BackL = hardwareMap.dcMotor.get("BackL");
        BackR = hardwareMap.dcMotor.get("BackR");

        Shooter2 = hardwareMap.dcMotor.get("Shooter2");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(RESET_X, RESET_Y, RESET_HEADING));
        follower.setPose(new Pose(RESET_X, RESET_Y, RESET_HEADING));
        follower.startTeleOpDrive();

        Servorot.setPosition(GATE_CLOSED);
        shooter.stop();

        shooterState = ShooterState.OFF;
        autoAimShooter.resetHeadingLock();
    }

    @Override
    public void loop() {

        boolean yPressed = gamepad1.y;
        boolean yJustPressed = yPressed && !lastYPressed;

        if (yJustPressed) {
            follower.setPose(new Pose(RESET_X, RESET_Y, RESET_HEADING));
            autoAimShooter.resetHeadingLock();
        }

        lastYPressed = yPressed;

        boolean aPressed = gamepad1.a;
        boolean aJustPressed = aPressed && !lastAPressed;

        if (aJustPressed) {
            safeMode = !safeMode;
            autoAimShooter.resetHeadingLock();
        }

        lastAPressed = aPressed;

        Pose pose = follower.getPose();

        double x = pose.getX();
        double y = pose.getY();
        double heading = pose.getHeading();

        boolean triggerPressed = gamepad1.left_trigger > TRIGGER_THRESHOLD;

        autoAimShooter.update(x, y, heading);

        isInAZone = autoAimShooter.isInSpinUpZone();
        autoAimActive = triggerPressed && isInAZone && !safeMode;

        ShooterSubsytem.TARGET_TPS = autoAimShooter.getTargetTPS();
        TARGET_TPS = ShooterSubsytem.TARGET_TPS;

        double turnInput;

        if (autoAimActive) {
            turnInput = autoAimShooter.getTurnPower();
        } else {
            turnInput = gamepad1.right_stick_x;
            autoAimShooter.resetHeadingLock();
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                turnInput,
                true
        );

        follower.update();

        if (gamepad1.right_bumper) {
            Intake.setPower(1);
            INTAKE.setPower(1);
        } else if (gamepad1.left_bumper) {
            Intake.setPower(-1);
            INTAKE.setPower(-1);
        } else {
            Intake.setPower(0);
            INTAKE.setPower(0);
        }

        if (isInAZone || safeMode || triggerPressed) {
            shooter.start();

            if (shooter.atSpeed()) {
                shooterState = ShooterState.READY;
            } else {
                shooterState = ShooterState.SPINNING_UP;
            }

        } else {
            shooter.stop();
            shooterState = ShooterState.OFF;
        }

        switch (shooterState) {

            case OFF:
                Servorot.setPosition(GATE_CLOSED);
                break;

            case SPINNING_UP:
                shooter.start();
                Servorot.setPosition(GATE_CLOSED);
                break;

            case READY:
                shooter.start();

                if (triggerPressed && (autoAimShooter.isAimed() || safeMode)) {
                    Servorot.setPosition(GATE_OPEN);
                } else {
                    Servorot.setPosition(GATE_CLOSED);
                }

                break;
        }

        shooter.update();

        telemetry.addLine("----- SAFE MODE -----");
        telemetry.addData("Safe Mode", safeMode);
        telemetry.addData("Toggle", "gamepad1.a");

        telemetry.addLine("----- POSE -----");
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading deg", Math.toDegrees(heading));

        telemetry.addLine("----- AUTO AIM -----");
        telemetry.addData("Auto Aim Active", autoAimActive);
        telemetry.addData("In Zone", isInAZone);
        telemetry.addData("Goal X", AutoAimShooter.GOAL_X);
        telemetry.addData("Goal Y", AutoAimShooter.GOAL_Y);
        telemetry.addData("Distance To Goal", autoAimShooter.getDistance());
        telemetry.addData("Angle To Goal Deg", Math.toDegrees(autoAimShooter.getAngleToGoal()));
        telemetry.addData("Turn Error Deg", Math.toDegrees(autoAimShooter.getTurnError()));
        telemetry.addData("Turn Power", autoAimShooter.getTurnPower());
        telemetry.addData("Aimed", autoAimShooter.isAimed());

        telemetry.addLine("----- SHOOTER -----");
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Trigger Pressed", triggerPressed);
        telemetry.addData("Target RPM", autoAimShooter.getTargetRPM());
        telemetry.addData("Target TPS", TARGET_TPS);
        telemetry.addData("Current TPS", shooter.getCurrentTPS());
        telemetry.addData("Error", shooter.getError());
        telemetry.addData("Power", shooter.getLastPower());
        telemetry.addData("Voltage", shooter.getVoltage());
        telemetry.addData("At Speed", shooter.atSpeed());
        telemetry.addData("Gate Position", Servorot.getPosition());

        telemetry.update();
    }
}