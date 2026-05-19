package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Drive Direction Test", group = "Diagnostics")
public class DriveDirectionTestTeleOp extends OpMode {
    public static double TEST_POWER = 0.25;
    public static double SLOW_POWER = 0.12;

    private Follower follower;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private int frontLeftZero;
    private int frontRightZero;
    private int backLeftZero;
    private int backRightZero;
    private boolean lastStart;

    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "FrontL");
        frontRight = hardwareMap.get(DcMotor.class, "FrontR");
        backLeft = hardwareMap.get(DcMotor.class, "BackL");
        backRight = hardwareMap.get(DcMotor.class, "BackR");

        applyPedroDriveDirections();
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        setRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        captureEncoderZeros();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.startTeleOpDrive();

        telemetry.addLine("D-pad: Pedro drive direction test");
        telemetry.addLine("A/B/X/Y: individual wheel test");
        telemetry.addLine("Hold left trigger for slower power");
        telemetry.addLine("Start: zero displayed encoder deltas");
        telemetry.update();
    }

    @Override
    public void loop() {
        double power = gamepad1.left_trigger > 0.2 ? SLOW_POWER : TEST_POWER;
        boolean individualTest = runIndividualWheelTest(power);

        if (!individualTest) {
            runPedroDirectionTest(power);
        }

        if (gamepad1.start && !lastStart) {
            captureEncoderZeros();
        }
        lastStart = gamepad1.start;

        showTelemetry(power, individualTest);
    }

    @Override
    public void stop() {
        setDrivePower(0, 0, 0, 0);
    }

    private void runPedroDirectionTest(double power) {
        double forward = 0;
        double strafe = 0;
        double turn = 0;

        if (gamepad1.dpad_up) {
            forward = power;
        } else if (gamepad1.dpad_down) {
            forward = -power;
        }

        if (gamepad1.dpad_right) {
            strafe = power;
        } else if (gamepad1.dpad_left) {
            strafe = -power;
        }

        if (gamepad1.right_bumper) {
            turn = power;
        } else if (gamepad1.left_bumper) {
            turn = -power;
        }

        follower.setTeleOpDrive(forward, strafe, turn, true);
        follower.update();
    }

    private boolean runIndividualWheelTest(double power) {
        double fl = gamepad1.a ? power : 0;
        double fr = gamepad1.b ? power : 0;
        double bl = gamepad1.x ? power : 0;
        double br = gamepad1.y ? power : 0;

        boolean anyWheelSelected = fl != 0 || fr != 0 || bl != 0 || br != 0;
        if (anyWheelSelected) {
            setDrivePower(fl, fr, bl, br);
        }

        return anyWheelSelected;
    }

    private void applyPedroDriveDirections() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
    }

    private void setRunMode(DcMotor.RunMode mode) {
        frontLeft.setMode(mode);
        frontRight.setMode(mode);
        backLeft.setMode(mode);
        backRight.setMode(mode);
    }

    private void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    private void captureEncoderZeros() {
        frontLeftZero = frontLeft.getCurrentPosition();
        frontRightZero = frontRight.getCurrentPosition();
        backLeftZero = backLeft.getCurrentPosition();
        backRightZero = backRight.getCurrentPosition();
    }

    private void showTelemetry(double power, boolean individualTest) {
        Pose pose = follower.getPose();

        telemetry.addData("Mode", individualTest ? "Individual wheel" : "Pedro drive");
        telemetry.addData("Power", power);
        telemetry.addLine("D-pad up/down = Pedro forward/back");
        telemetry.addLine("D-pad right/left = Pedro strafe right/left");
        telemetry.addLine("RB/LB = Pedro positive/negative turn");
        telemetry.addLine("A=FrontL  B=FrontR  X=BackL  Y=BackR");
        telemetry.addData("FrontL direction", frontLeft.getDirection());
        telemetry.addData("FrontR direction", frontRight.getDirection());
        telemetry.addData("BackL direction", backLeft.getDirection());
        telemetry.addData("BackR direction", backRight.getDirection());
        telemetry.addData("FrontL delta", frontLeft.getCurrentPosition() - frontLeftZero);
        telemetry.addData("FrontR delta", frontRight.getCurrentPosition() - frontRightZero);
        telemetry.addData("BackL delta", backLeft.getCurrentPosition() - backLeftZero);
        telemetry.addData("BackR delta", backRight.getCurrentPosition() - backRightZero);
        telemetry.addData("Pedro X", pose.getX());
        telemetry.addData("Pedro Y", pose.getY());
        telemetry.addData("Pedro heading deg", Math.toDegrees(pose.getHeading()));
        telemetry.update();
    }
}
