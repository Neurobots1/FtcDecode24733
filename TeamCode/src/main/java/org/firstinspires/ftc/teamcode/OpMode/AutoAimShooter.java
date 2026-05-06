package org.firstinspires.ftc.teamcode.OpMode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.ElapsedTime;

// Systeme simple sans camera: utilise la pose Pedro pour viser le but et choisir la vitesse du shooter.
@Configurable
public class AutoAimShooter {

    // Position du but en coordonnees terrain Pedro, a ajuster selon ton terrain.
    public static double GOAL_X = 3;
    public static double GOAL_Y = 141;

    // Meme logique que le heading lock du projet FTC-2025-2026: PID + petit feed-forward.
    public static double CHASSIS_AIM_KP = 0.8;
    public static double CHASSIS_AIM_KI = 0.0;
    public static double CHASSIS_AIM_KD = 0.0;
    public static double CHASSIS_AIM_KF = 0.0;
    public static double MAX_TURN_POWER = 0.4;
    public static double AIM_TOLERANCE_DEG = 2.0;

    // Conversion RPM -> ticks/sec, parce que ShooterSubsytem controle getVelocity() en TPS.
    public static double FLYWHEEL_ENCODER_TICKS_PER_REV = 28.0;

    // Table de calibration: {distance en pouces, vitesse flywheel en RPM}.
    // Ajoute/ajuste les points ici; le code interpole lineairement entre deux points.
    private static final double[][] DISTANCE_RPM_TABLE = {
            {24.0, 2350.0},
            {36.0, 2575.0},
            {48.0, 2800.0},
            {60.0, 3025.0},
            {72.0, 3250.0},
            {84.0, 3475.0}
    };

    private final ElapsedTime headingTimer = new ElapsedTime();

    private double distance;
    private double angleToGoal;
    private double turnError;
    private double turnPower;
    private double targetRPM;
    private double targetTPS;
    private boolean aimed;
    private boolean inSpinUpZone;

    private double integral;
    private double previousError;
    private double lastProportionalTerm;
    private double lastIntegralTerm;
    private double lastDerivativeTerm;
    private double lastFeedforwardTerm;

    public void update(double robotX, double robotY, double robotHeading) {
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;

        distance = Math.sqrt(dx * dx + dy * dy);
        angleToGoal = Math.atan2(dy, dx);
        turnError = normalizeAngle(angleToGoal - robotHeading);
        inSpinUpZone = isInsideTable(distance);

        updateHeadingLock();

        targetRPM = getShooterRPM(distance);
        targetTPS = rpmToTicksPerSecond(targetRPM);
        aimed = Math.abs(turnError) < Math.toRadians(AIM_TOLERANCE_DEG);
    }

    public void resetHeadingLock() {
        integral = 0.0;
        previousError = 0.0;
        turnPower = 0.0;
        lastProportionalTerm = 0.0;
        lastIntegralTerm = 0.0;
        lastDerivativeTerm = 0.0;
        lastFeedforwardTerm = 0.0;
        headingTimer.reset();
    }

    public double getTurnPower() {
        return turnPower;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getTargetTPS() {
        return targetTPS;
    }

    public double getDistance() {
        return distance;
    }

    public double getTurnError() {
        return turnError;
    }

    public boolean isAimed() {
        return aimed;
    }

    public boolean isInSpinUpZone() {
        return inSpinUpZone;
    }

    public double getLastProportionalTerm() {
        return lastProportionalTerm;
    }

    public double getLastIntegralTerm() {
        return lastIntegralTerm;
    }

    public double getLastDerivativeTerm() {
        return lastDerivativeTerm;
    }

    public double getLastFeedforwardTerm() {
        return lastFeedforwardTerm;
    }

    private void updateHeadingLock() {
        double dt = Math.max(1e-3, headingTimer.seconds());
        headingTimer.reset();

        lastProportionalTerm = turnError * CHASSIS_AIM_KP;
        lastDerivativeTerm = ((turnError - previousError) / dt) * CHASSIS_AIM_KD;
        lastFeedforwardTerm = Math.abs(turnError) > 1e-4
                ? Math.signum(turnError) * CHASSIS_AIM_KF
                : 0.0;

        double candidateIntegral = integral + turnError * dt;
        double candidateIntegralTerm = candidateIntegral * CHASSIS_AIM_KI;
        double unclampedCommand = lastProportionalTerm
                + candidateIntegralTerm
                + lastDerivativeTerm
                + lastFeedforwardTerm;

        if (Math.abs(unclampedCommand) <= MAX_TURN_POWER
                || Math.signum(unclampedCommand) != Math.signum(turnError)) {
            integral = candidateIntegral;
        }

        lastIntegralTerm = integral * CHASSIS_AIM_KI;
        previousError = turnError;
        turnPower = clip(
                lastProportionalTerm
                        + lastIntegralTerm
                        + lastDerivativeTerm
                        + lastFeedforwardTerm,
                -MAX_TURN_POWER,
                MAX_TURN_POWER
        );
    }

    private double getShooterRPM(double distance) {
        if (DISTANCE_RPM_TABLE.length == 0) {
            return 0.0;
        }
        if (distance <= DISTANCE_RPM_TABLE[0][0]) {
            return DISTANCE_RPM_TABLE[0][1];
        }
        if (distance >= DISTANCE_RPM_TABLE[DISTANCE_RPM_TABLE.length - 1][0]) {
            return DISTANCE_RPM_TABLE[DISTANCE_RPM_TABLE.length - 1][1];
        }

        for (int i = 0; i < DISTANCE_RPM_TABLE.length - 1; i++) {
            double d1 = DISTANCE_RPM_TABLE[i][0];
            double rpm1 = DISTANCE_RPM_TABLE[i][1];
            double d2 = DISTANCE_RPM_TABLE[i + 1][0];
            double rpm2 = DISTANCE_RPM_TABLE[i + 1][1];

            if (distance >= d1 && distance <= d2) {
                double t = (distance - d1) / (d2 - d1);
                return lerp(rpm1, rpm2, t);
            }
        }

        return DISTANCE_RPM_TABLE[DISTANCE_RPM_TABLE.length - 1][1];
    }

    private boolean isInsideTable(double distance) {
        return DISTANCE_RPM_TABLE.length > 0
                && distance >= DISTANCE_RPM_TABLE[0][0]
                && distance <= DISTANCE_RPM_TABLE[DISTANCE_RPM_TABLE.length - 1][0];
    }

    private double rpmToTicksPerSecond(double rpm) {
        return rpm * FLYWHEEL_ENCODER_TICKS_PER_REV / 60.0;
    }

    private double lerp(double a, double b, double t) {
        return a + (b - a) * clip(t, 0.0, 1.0);
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }
        return angle;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
