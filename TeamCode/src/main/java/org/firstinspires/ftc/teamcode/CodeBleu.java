
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ShooterSubsytem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@TeleOp
public class CodeBleu extends OpMode {

    public ShooterSubsytem shooter;
    private boolean safeMode = false;
    private boolean lastAPressed = false;
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

    // Mets 30 pour tester plus facilement, puis redescends à 10 après
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
        follower.setStartingPose(new Pose(RESET_X, RESET_Y, RESET_HEADING));
        follower.setPose(new Pose(RESET_X, RESET_Y, RESET_HEADING));
        follower.startTeleOpDrive();

        Servorot.setPosition(GATE_CLOSED);
        shooter.stop();

        shooterState = ShooterState.OFF;
    }

    public boolean isInBackZone(double x, double y, double radiusInches) {
        double d = radiusInches * Math.sqrt(2);

        boolean underFirstLine = y <= x - 48 + d;
        boolean underSecondLine = y <= -x + 96 + d;

        return underFirstLine && underSecondLine;
    }

    public boolean isInFrontZone(double x, double y, double radiusInches) {
        double d = radiusInches * Math.sqrt(2);

        boolean aboveFirstLine = y >= -x + 144 - d;
        boolean aboveSecondLine = y >= x - d;

        return aboveFirstLine && aboveSecondLine;
    }

    @Override
    public void loop() {

        // Reset la pose avec Y
        boolean yPressed = gamepad1.y;
        boolean yJustPressed = yPressed && !lastYPressed;

        if (yJustPressed) {
            follower.setPose(new Pose(RESET_X, RESET_Y, RESET_HEADING));
        }

        lastYPressed = yPressed;

        // Drive Pedro
        follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                true
        );

        // Important : update après setTeleOpDrive
        follower.update();

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

        // Pose du robot
        Pose pose = follower.getPose();

        double x = pose.getX();
        double y = pose.getY();

        // Calcul des lignes pour debug
        double d = ZONE_RADIUS_INCHES * Math.sqrt(2);

        double backLine1 = x - 48 + d;
        double backLine2 = -x + 96 + d;

        double frontLine1 = -x + 144 - d;
        double frontLine2 = x - d;

        // Conditions individuelles pour voir ce qui bloque
        boolean backCondition1 = y <= backLine1;
        boolean backCondition2 = y <= backLine2;

        boolean frontCondition1 = y >= frontLine1;
        boolean frontCondition2 = y >= frontLine2;

        boolean inBackZone = backCondition1 && backCondition2;
        boolean inFrontZone = frontCondition1 && frontCondition2;

        isInAZone = inFrontZone || inBackZone;

        boolean triggerPressed = gamepad1.left_trigger > TRIGGER_THRESHOLD;

        // Shooter automatique dans la zone
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

        telemetry.addLine("----- POSE -----");
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("heading", pose.getHeading());

        telemetry.addLine("----- ZONE DEBUG -----");
        telemetry.addData("Zone Radius", ZONE_RADIUS_INCHES);
        telemetry.addData("d", d);

        telemetry.addData("Back Line 1 : y <= ", backLine1);
        telemetry.addData("Back Line 2 : y <= ", backLine2);
        telemetry.addData("Back Condition 1", backCondition1);
        telemetry.addData("Back Condition 2", backCondition2);
        telemetry.addData("In Back Zone", inBackZone);

        telemetry.addData("Front Line 1 : y >= ", frontLine1);
        telemetry.addData("Front Line 2 : y >= ", frontLine2);
        telemetry.addData("Front Condition 1", frontCondition1);
        telemetry.addData("Front Condition 2", frontCondition2);
        telemetry.addData("In Front Zone", inFrontZone);

        telemetry.addData("___In Any Zone", isInAZone);

        telemetry.addLine("----- SHOOTER -----");
        telemetry.addData("Shooter State", shooterState);
        telemetry.addData("Trigger Pressed", triggerPressed);
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
