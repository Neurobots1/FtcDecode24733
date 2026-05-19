package org.firstinspires.ftc.teamcode;

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
public class StateMachineTele extends OpMode {

    public ShooterSubsytem shooter;
    private Follower follower;
    private AutoAimShooter autoAim;

    private Servo Servorot;
    private DcMotor FrontL;
    private DcMotor FrontR;
    private DcMotor BackL;
    private DcMotor BackR;
    private DcMotor Shooter2;
    private DcMotor Intake;
    private DcMotor INTAKE;

    public double TARGET_TPS = 1360;
    public double DISTANCE = 8;

    public double servo_cool = 0.4;
    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));
    public static double GATE_OPEN = 0.69;
    public static double GATE_CLOSED = 0.0;
    public static double TRIGGER_THRESHOLD = 0.1;
    public static double DRIVER_TURN_OVERRIDE = 0.08;
    public static double MAIN_FEED_INTAKE_POWER = 1.0;

    public enum ShooterState {
        OFF,
        SPINNING_UP,
        FIRING
    }

    private ShooterState shooterState = ShooterState.OFF;
    private boolean fireToggleLast = false;
    private boolean shootingArmed = false;
    private boolean gateLatchedOpen = false;

    @Override
    public void init() {
        shooter = new ShooterSubsytem(hardwareMap);
        autoAim = new AutoAimShooter();

        Servorot = hardwareMap.get(Servo.class, "Servorot");
        INTAKE = hardwareMap.dcMotor.get("INTAKE");
        Intake = hardwareMap.dcMotor.get("Intake");
        FrontL = hardwareMap.dcMotor.get("FrontL");
        FrontR = hardwareMap.dcMotor.get("FrontR");
        BackL = hardwareMap.dcMotor.get("BackL");
        BackR = hardwareMap.dcMotor.get("BackR");
        Shooter2 = hardwareMap.dcMotor.get("Shooter2");

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.update();

        Servorot.setPosition(GATE_CLOSED);
        shooter.stop();
        autoAim.resetHeadingLock();
        shooterState = ShooterState.OFF;
        shootingArmed = false;
        gateLatchedOpen = false;
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
        follower.setStartingPose(startPose);
        follower.update();
        autoAim.resetHeadingLock();
        stopShootingSequence();
    }

    @Override
    public void loop() {
        Pose robotPose = follower.getPose();
        autoAim.update(robotPose.getX(), robotPose.getY(), robotPose.getHeading());
        shooter.setTargetTPS(autoAim.getTargetTPS());

        double driverTurnCommand = -gamepad1.right_stick_x;
        double turnCommand = driverTurnCommand;
        boolean headingLockActive = shootingArmed
                && autoAim.isInSpinUpZone()
                && Math.abs(driverTurnCommand) < DRIVER_TURN_OVERRIDE;
        if (headingLockActive) {
            turnCommand = autoAim.getTurnPower();
        } else {
            autoAim.resetHeadingLock();
        }

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turnCommand,
                false,
                Math.toRadians(180));
        follower.update();
        robotPose = follower.getPose();

        if (gamepad1.y && !fireToggleLast) {
            shootingArmed = !shootingArmed;
            if (shootingArmed) {
                shooterState = ShooterState.SPINNING_UP;
            } else {
                stopShootingSequence();
            }
        }
        fireToggleLast = gamepad1.y;

        DISTANCE = autoAim.getDistance();
        TARGET_TPS = autoAim.getTargetTPS();
        boolean autoSpinRequested = autoAim.isInSpinUpZone();
        boolean shooterReady = shooter.atSpeed() && autoAim.isAimed();

        if (shootingArmed && !autoSpinRequested) {
            stopShootingSequence();
        }

        switch (shooterState) {
            case OFF:
                Servorot.setPosition(GATE_CLOSED);
                gateLatchedOpen = false;
                if (autoSpinRequested) {
                    shooter.start();
                } else {
                    shooter.stop();
                }
                break;

            case SPINNING_UP:
                shooter.start();
                Servorot.setPosition(GATE_CLOSED);
                if (shootingArmed && shooterReady) {
                    gateLatchedOpen = true;
                    shooterState = ShooterState.FIRING;
                }
                break;

            case FIRING:
                shooter.start();
                if (shootingArmed && gateLatchedOpen) {
                    Servorot.setPosition(GATE_OPEN);
                } else {
                    Servorot.setPosition(GATE_CLOSED);
                    gateLatchedOpen = false;
                    shooterState = shootingArmed ? ShooterState.SPINNING_UP : ShooterState.OFF;
                }
                break;
        }

        boolean gateOpen = shooterState == ShooterState.FIRING && gateLatchedOpen;
        boolean forceFeedIntake = gateOpen;

        if (forceFeedIntake) {
            setIntakePower(MAIN_FEED_INTAKE_POWER);
        } else if (gamepad1.right_bumper) {
            setIntakePower(1);
        } else if (gamepad1.left_bumper) {
            setIntakePower(-1);
        } else {
            setIntakePower(0);
        }

        shooter.update();

        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Shooting Armed", shootingArmed);
        telemetry.addData("Gate Latched", gateLatchedOpen);
        telemetry.addData("Force Feed Intake", forceFeedIntake);
        telemetry.addData("Target TPS", TARGET_TPS);
        telemetry.addData("Target RPM", autoAim.getTargetRPM());
        telemetry.addData("Current TPS", shooter.getCurrentTPS());
        telemetry.addData("Error", shooter.getError());
        telemetry.addData("Power", shooter.getLastPower());
        telemetry.addData("Voltage", shooter.getVoltage());
        telemetry.addData("At Speed", shooter.atSpeed());
        telemetry.addData("x:", robotPose.getX());
        telemetry.addData("y:", robotPose.getY());
        telemetry.addData("heading deg:", Math.toDegrees(robotPose.getHeading()));
        telemetry.addData("Distance:", DISTANCE);
        telemetry.addData("In Spin Zone", autoAim.isInSpinUpZone());
        telemetry.addData("Aimed", autoAim.isAimed());
        telemetry.addData("Heading Lock", headingLockActive);
        telemetry.addData("Heading Error Deg", Math.toDegrees(autoAim.getTurnError()));
        telemetry.addData("Turn Command", turnCommand);
        telemetry.addData("servo:position", servo_cool);
        telemetry.update();
    }

    private void setIntakePower(double power) {
        Intake.setPower(power);
        INTAKE.setPower(power);
    }

    private void stopShootingSequence() {
        shootingArmed = false;
        shooterState = ShooterState.OFF;
        gateLatchedOpen = false;
        Servorot.setPosition(GATE_CLOSED);
    }
}
