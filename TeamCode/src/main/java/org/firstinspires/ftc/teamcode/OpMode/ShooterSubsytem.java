package org.firstinspires.ftc.teamcode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
public class ShooterSubsytem {

    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;
    private final VoltageSensor voltageSensor;

    public static double P = 0.002;
    public static double I = 0.0;
    public static double D = 0.00001;
    public static double F = 0.00038;

    public static double TARGET_TPS = 1400;
    public static double VELOCITY_TOLERANCE_TPS = 50;
    public static double NOMINAL_VOLTAGE = 12;
    public static double MAX_POWER = 1.0;

    private boolean enabled = false;

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private double lastPower = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    public ShooterSubsytem(HardwareMap hardwareMap) {
        shooter1 = hardwareMap.get(DcMotorEx.class, "Shooter1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "Shooter2");
        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

        timer.reset();
    }

    public void start() {
        enabled = true;
    }

    public void stop() {
        enabled = false;
        integralSum = 0.0;
        lastError = 0.0;
        lastPower = 0.0;
        shooter1.setPower(0.0);
        shooter2.setPower(0.0);
    }

    public void update() {
        if (!enabled || TARGET_TPS <= 0) {
            shooter1.setPower(0.0);
            shooter2.setPower(0.0);
            integralSum = 0.0;
            lastError = 0.0;
            lastPower = 0.0;
            timer.reset();
            return;
        }

        double currentTPS = getCurrentTPS();
        double error = TARGET_TPS - currentTPS;

        double dt = timer.seconds();
        timer.reset();

        if (dt <= 0) dt = 0.001;

        integralSum += error * dt;
        double derivative = (error - lastError) / dt;
        lastError = error;

        double voltage = voltageSensor.getVoltage();
        double normalizedVoltage = voltage / NOMINAL_VOLTAGE;
        if (normalizedVoltage <= 0) normalizedVoltage = 1.0;

        double pid = (P * error) + (I * integralSum) + (D * derivative);
        double ff = F * TARGET_TPS / normalizedVoltage;

        double power = pid + ff;
        power = Math.max(0.0, Math.min(MAX_POWER, power));

        lastPower = power;

        shooter1.setPower(power);
        shooter2.setPower(power);
    }

    public boolean atSpeed() {
        return enabled && TARGET_TPS > 0 && Math.abs(TARGET_TPS - getCurrentTPS()) <= VELOCITY_TOLERANCE_TPS;
    }

    public double getCurrentTPS() {
        return Math.abs(shooter1.getVelocity());
    }

    public double getCurrentTPSMotor1() {
        return Math.abs(shooter1.getVelocity());
    }

    public double getCurrentTPSMotor2() {
        return Math.abs(shooter2.getVelocity());
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    public double getLastPower() {
        return lastPower;
    }

    public double getError() {
        return TARGET_TPS - getCurrentTPS();
    }

    public boolean isEnabled() {
        return enabled;
    }
}