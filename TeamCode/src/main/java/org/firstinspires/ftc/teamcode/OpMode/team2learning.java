package org.firstinspires.ftc.teamcode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
@TeleOp
public class team2learning extends OpMode {

    private DcMotor FrontL;
    private DcMotor FrontR;
    private DcMotor BackL;
    private DcMotor BackR;

    @Override
    public void init() {
        FrontL = hardwareMap.dcMotor.get("FrontL");
        FrontR = hardwareMap.dcMotor.get("FrontR");
        BackL = hardwareMap.dcMotor.get("BackL");
        BackR = hardwareMap.dcMotor.get("BackR");
    }
     @Override
    public void loop() {

        if (gamepad1.dpad_left) {
            if (gamepad1.left_stick_y < 0.1) {
                if (gamepad1.left_stick_x < 0.1) {
                    FrontL.setPower((gamepad1.left_stick_x / 2) + (gamepad1.left_stick_y / 2));
                }
            }

        }
        else {
            if (gamepad1.left_stick_y > 0.1) {
                FrontL.setPower(gamepad1.left_stick_y);
                FrontR.setPower(gamepad1.left_stick_y);
                BackL.setPower(gamepad1.left_stick_y);
                BackR.setPower(gamepad1.left_stick_y);
            }
            if (gamepad1.left_stick_y < -0.1) {
                FrontL.setPower(gamepad1.left_stick_y);
                FrontR.setPower(gamepad1.left_stick_y);
                BackL.setPower(gamepad1.left_stick_y);
                BackR.setPower(gamepad1.left_stick_y);
            }
            if (gamepad1.left_stick_x > 0.1) {
                FrontL.setPower(gamepad1.left_stick_x);
                FrontR.setPower(gamepad1.left_stick_x);
                BackL.setPower(gamepad1.left_stick_x);
                BackR.setPower(gamepad1.left_stick_x);
            }
            if (gamepad1.left_stick_x < -0.1) {
                FrontL.setPower(gamepad1.left_stick_x);
                FrontR.setPower(gamepad1.left_stick_x);
                BackL.setPower(gamepad1.left_stick_x);
                BackR.setPower(gamepad1.left_stick_x);
            }
            if (gamepad1.right_stick_x > 0.1) {
                FrontL.setPower(gamepad1.right_stick_x);
                FrontR.setPower(-gamepad1.right_stick_x);
                BackL.setPower(-gamepad1.right_stick_x);
                BackR.setPower(gamepad1.right_stick_x);
            }
            if (gamepad1.right_stick_x < -0.1) {
                FrontL.setPower(gamepad1.right_stick_x);
                FrontR.setPower(-gamepad1.right_stick_x);
                BackL.setPower(-gamepad1.right_stick_x);
                BackR.setPower(gamepad1.right_stick_x);
            }
        }
     }
}