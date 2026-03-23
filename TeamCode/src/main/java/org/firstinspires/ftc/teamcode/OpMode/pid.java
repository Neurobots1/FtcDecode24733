package org.firstinspires.ftc.teamcode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotserver.internal.webserver.controlhubupdater.ChUpdaterCommManager;

@Configurable
@TeleOp
public class pid extends OpMode {
    private DcMotorEx Shooter1;
    private DcMotorEx Shooter2;
    public  static double TargetRpm = 0; //creer la variable que modifie le rpm


    public ShooterSubsytem shooter; // appel le subsystem du shooter
    @Override
    public void init() {
        shooter = new ShooterSubsytem(hardwareMap); //creer le hardwaremap du shooter avec ses variable de pid

    }

    @Override
    public void loop() {


        if (gamepad1.a){
            TargetRpm = 1000; //modifie la valeur du Target Rpm
        }

        if (gamepad1.b){
            TargetRpm = 0; //modifie la valeur du Target Rpm
        }

        //shooter.setTargetVelocity(TargetRpm); //dit au shooter a quelle vitesse tourne
        shooter.update(); // update le shooter
    }

}
