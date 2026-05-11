package org.firstinspires.ftc.teamcode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;




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
public class CodeRouge extends OpMode {

    public ShooterSubsytem shooter;
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
    public static double TARGET_TPS = 900;

    public static double GATE_OPEN = 0.69;
    public static double GATE_CLOSED = 0.0;

    public static double TRIGGER_THRESHOLD = 0.05;

    public static double ZONE_RADIUS_INCHES = 10;

    public boolean isInAZone;

    public enum ShooterState {
        OFF,
        SPINNING_UP,
        READY
    }

    private ShooterState shooterState = ShooterState.OFF;

    @Override
    public void init() {
        shooter = new ShooterSubsytem(hardwareMap);

        Servorot = hardwareMap.get(Servo.class, "Servorot");

        INTAKE = hardwareMap.dcMotor.get("INTAKE");
        Intake = hardwareMap.dcMotor.get("Intake");

        FrontL = hardwareMap.dcMotor.get("FrontL");
        FrontR = hardwareMap.dcMotor.get("FrontR");
        BackL = hardwareMap.dcMotor.get("BackL");
        BackR = hardwareMap.dcMotor.get("BackR");

        Shooter2 = hardwareMap.dcMotor.get("Shooter2");

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleOpDrive();
        follower.update();

        Servorot.setPosition(GATE_CLOSED);
        shooter.stop();

        shooterState = ShooterState.OFF;
    }

    public boolean isInBackZone(double x, double y, double radiusInches) {
        double d = radiusInches * 1.41421356237;

        return y < (x - 48 + d) && y <= (-x + 96 + d);
    }

    public boolean isInFrontZone(double x, double y, double radiusInches) {
        double d = radiusInches * 1.41421356237;

        return y >= -x + 144 - d && y >= x - d;
    }

    @Override
    public void loop() {
        follower.update();
        boolean yPressed = gamepad1.y;
        boolean yJustPressed = yPressed && !lastYPressed;

        if (yJustPressed) {
            follower.setPose(new Pose(RESET_X, RESET_Y, RESET_HEADING));
        }

        lastYPressed = yPressed;
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                true
        );

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

        double x = follower.getPose().getX();
        double y = follower.getPose().getY();

        boolean inFrontZone = isInFrontZone(x, y, ZONE_RADIUS_INCHES);
        boolean inBackZone = isInBackZone(x, y, ZONE_RADIUS_INCHES);

        isInAZone = inFrontZone || inBackZone;

        boolean triggerPressed = gamepad1.left_trigger > TRIGGER_THRESHOLD;

        if (isInAZone) {
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

                if (triggerPressed) {
                    Servorot.setPosition(GATE_OPEN);
                } else {
                    Servorot.setPosition(GATE_CLOSED);
                }

                break;
        }

        shooter.update();

        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("In Front Zone", inFrontZone);
        telemetry.addData("In Back Zone", inBackZone);
        telemetry.addData("In Any Zone", isInAZone);
        telemetry.addData("Trigger Pressed", triggerPressed);
        telemetry.addData("Target TPS", TARGET_TPS);
        telemetry.addData("Current TPS", shooter.getCurrentTPS());
        telemetry.addData("Error", shooter.getError());
        telemetry.addData("Power", shooter.getLastPower());
        telemetry.addData("Voltage", shooter.getVoltage());
        telemetry.addData("At Speed", shooter.atSpeed());
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("Gate Position", Servorot.getPosition());

        telemetry.update();
    }
}