package org.firstinspires.ftc.teamcode.OpMode;

import static org.firstinspires.ftc.teamcode.OpMode.ShooterSubsytem.TARGET_TPS;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.OpMode.ShooterSubsytem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class StateMachineTele extends OpMode {

    public ShooterSubsytem shooter;//fais appelle au Code ShooterSubsystem
    private Follower follower;//Pedropathing follower
    private Servo Servorot;//défine le servo en tant qu'existant
    private DcMotor FrontL;//22 à 27 dit les moteurs
    private DcMotor FrontR;
    private DcMotor BackL;
    private DcMotor BackR;
    private DcMotor Shooter1;
    private DcMotor Shooter2;

    private DcMotor Intake;
    private DcMotor INTAKE;

    //défini les positions du shooters
    private final Pose startPose = new Pose(72, 72, Math.toRadians(90));
    public static double GATE_OPEN = 0.0;
    public static double GATE_CLOSED = 0.89;
    //dis au code quand le trigger est triggerrer pouv éviter les missclicks
    public static double TRIGGER_THRESHOLD = 0.1;

    //défini les states de la state machine afin que le code sache qu'elle existe
    public enum ShooterState {
        OFF,          // shooter arrêté
        SPINNING_UP,  // flywheel en train d'accélérer
        FIRING        // flywheel prêt, gate ouvert

    }

    //dis au code qu'au début il est pas actif pour éviter les bugs (fais appelle au premier state ligne 41)
    private ShooterState shooterState = ShooterState.OFF;

    // permettera au code de pouvoir cliquer la gachette et non la maintenir
    private boolean lastLeftTriggerPressed = false;


    @Override
    public void init() {

        shooter = new ShooterSubsytem(hardwareMap);
        //dis au code que le Shooter Subsytem s'appelle shooter il sera nommé shooter dans le code
        Servorot = hardwareMap.get(Servo.class, "Servorot");
        //le servo est nommer Servorot et il est un servo class il pourrait être un servorot class
        INTAKE = hardwareMap.dcMotor.get("INTAKE");
        Intake = hardwareMap.dcMotor.get("Intake");
        FrontL = hardwareMap.dcMotor.get("FrontL");
        FrontR = hardwareMap.dcMotor.get("FrontR");
        BackL = hardwareMap.dcMotor.get("BackL");
        BackR = hardwareMap.dcMotor.get("BackR");
        Shooter1 = hardwareMap.dcMotor.get("Shooter1");
        Shooter2 = hardwareMap.dcMotor.get("Shooter2");
        //nomme tous les moteurs
        Shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // explique au code que le shooter utilise l'encondeur ce qui fais le PID du ShooterSubsytem
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        follower.startTeleOpDrive();
        follower.update();

        //dis au code que au début le shooter et le servo sont a off
        Servorot.setPosition(GATE_CLOSED);
        shooter.stop();
        shooterState = ShooterState.OFF;
    }

    @Override
    public void loop() {
        // les followwer updates et les gamepad1.Xstick_variable x ou y
        // disent au robot à l'aide de pedro path que le robot roule en mechanum
        follower.update();
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false,
                90);
        // contrôle intake
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

        //le TARGET_TPS c'est le shooter
        TARGET_TPS = 1360;

        //définie l'état du trigger pour le shooter (activer ou pas sans avoir a maintenir le trigger
        // position active ou pas
        boolean leftTriggerPressed = gamepad1.left_trigger > TRIGGER_THRESHOLD;
        boolean leftTriggerJustPressed = leftTriggerPressed && !lastLeftTriggerPressed;
//transmet l'état de la gachette en mouvement pour le shooter
        if (leftTriggerJustPressed) {

            if (shooterState == ShooterState.OFF) {

                // premier clic
                // on commence le spinup
                shooterState = ShooterState.SPINNING_UP;

                Servorot.setPosition(GATE_CLOSED); // gate fermé
                shooter.start();                   // flywheel démarre

            } else {

                // deuxième clic
                // arrêt complet du système
                shooterState = ShooterState.OFF;

                Servorot.setPosition(GATE_CLOSED);
                shooter.stop();
            }
        }

        //défini chaque state de la state machine du shooter
        switch (shooterState) {

            case OFF:

                // shooter arrêté et servo bloquant
                Servorot.setPosition(GATE_CLOSED);
                shooter.stop();

                break;

            case SPINNING_UP:

                // flywheel accélère ET le servo reste fermer
                shooter.start();
                Servorot.setPosition(GATE_CLOSED);

                // si LA vitesse est atteinte on passe en firing mode
                if (shooter.atSpeed()) {

                    shooterState = ShooterState.FIRING;

                }

                break;

            case FIRING:

                // shooter reste en mode tir et le servo reste ouvert
                shooter.start();
                Servorot.setPosition(GATE_OPEN);

                break;
        }

        shooter.update();

        //dis de la telemetry au robot
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Target TPS", TARGET_TPS);
        telemetry.addData("Current TPS", shooter.getCurrentTPS());
        telemetry.addData("Error", shooter.getError());
        telemetry.addData("Power", shooter.getLastPower());
        telemetry.addData("Voltage", shooter.getVoltage());
        telemetry.addData("At Speed", shooter.atSpeed());

        telemetry.update();
    }

}