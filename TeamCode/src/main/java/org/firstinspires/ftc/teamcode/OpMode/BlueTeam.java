package org.firstinspires.ftc.teamcode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class BlueTeam extends OpMode {

    public ShooterSubsytem shooter;

    private Follower follower;

    private Servo Servorot;

    private DcMotor FrontL;
    private DcMotor FrontR;
    private DcMotor BackL;
    private DcMotor BackR;

    private DcMotor Shooter1;
    private DcMotor Shooter2;

    private DcMotor Intake;
    private DcMotor INTAKE;

    // Position de départ du robot
    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));

    // Position du but sur le terrain
    // À ajuster selon la vraie position du but dans Pedro Pathing
    public static double GOAL_X = 0;
    public static double GOAL_Y = 144;

    // Réglage du shooter
    public double TARGET_TPS = 1360;
    public double DISTANCE = 10;

    // Formule temporaire :
    // TARGET_TPS = BASE_TPS + DISTANCE * TPS_PER_INCH
    // Tu vas devoir tuner ces valeurs avec des tests réels.
    public static double BASE_TPS = 1000;
    public static double TPS_PER_INCH = 2;

    // Gate du shooter
    public static double GATE_OPEN = 0.66;
    public static double GATE_CLOSED = 0.0;

    public enum ShooterState {
        OFF,
        SPINNING_UP,
        FIRING
    }

    private ShooterState shooterState = ShooterState.OFF;

    private boolean lastLeftTriggerPressed = false;

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

        Shooter1 = hardwareMap.dcMotor.get("Shooter1");
        Shooter2 = hardwareMap.dcMotor.get("Shooter2");

        Shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleOpDrive();

        Servorot.setPosition(GATE_CLOSED);
        shooter.stop();
        shooterState = ShooterState.OFF;
    }

    @Override
    public void loop() {

        follower.update();

        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                true,
                Math.toRadians(180)
        );

        // Intake
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

        // Calcul automatique distance + vitesse shooter
        Pose pose = follower.getPose();

        double dx = GOAL_X - pose.getX();
        double dy = GOAL_Y - pose.getY();

        DISTANCE = Math.sqrt(dx * dx + dy * dy);

        TARGET_TPS = Math.round(BASE_TPS + TPS_PER_INCH * DISTANCE);

        // IMPORTANT :
        // Cette ligne suppose que ton ShooterSubsytem possède une méthode setTargetTPS(double).
        // Si elle n’existe pas encore, il faut l’ajouter dans ShooterSubsytem.
        shooter.setTargetTPS(TARGET_TPS);


        // Trigger toggle pour shooter
        boolean leftTriggerPressed = gamepad1.left_trigger != 0;
        boolean leftTriggerJustPressed = leftTriggerPressed && !lastLeftTriggerPressed;

        if (leftTriggerJustPressed) {

            if (shooterState == ShooterState.OFF) {

                shooterState = ShooterState.SPINNING_UP;

                Servorot.setPosition(GATE_CLOSED);
                shooter.start();

            } else {

                shooterState = ShooterState.OFF;

                Servorot.setPosition(GATE_CLOSED);
                shooter.stop();
            }
        }

        lastLeftTriggerPressed = leftTriggerPressed;

        // State machine du shooter
        switch (shooterState) {

            case OFF:

                Servorot.setPosition(GATE_CLOSED);
                shooter.stop();

                break;

            case SPINNING_UP:

                shooter.start();
                Servorot.setPosition(GATE_CLOSED);

                if (shooter.atSpeed()) {
                    shooterState = ShooterState.FIRING;
                }

                break;

            case FIRING:

                shooter.start();
                Servorot.setPosition(GATE_OPEN);

                break;
        }

        shooter.update();

        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Target TPS", TARGET_TPS);
        telemetry.addData("Current TPS", shooter.getCurrentTPS());
        telemetry.addData("Error", shooter.getError());
        telemetry.addData("Power", shooter.getLastPower());

        telemetry.addData("Robot X", pose.getX());
        telemetry.addData("Robot Y", pose.getY());
        telemetry.addData("Robot Heading", Math.toDegrees(pose.getHeading()));

        telemetry.addData("Goal X", GOAL_X);
        telemetry.addData("Goal Y", GOAL_Y);
        telemetry.addData("Distance", DISTANCE);

        telemetry.update();
    }
}