package org.firstinspires.ftc.teamcode.OpMode;

// AutoAimShooter.java
// Système simple sans caméra : utilise la pose Pedro pour viser le but et choisir la vitesse du shooter.

public class AutoAimShooter {

    // Position du but en coordonnées terrain Pedro, à ajuster selon ton terrain
    private static final double GOAL_X = 3;
    private static final double GOAL_Y = 141;

    // Ajustements pour viser le but
    private static final double KP_TURN = 0.8;
    private static final double MAX_TURN_POWER = 0.4;
    private static final double AIM_TOLERANCE_RAD = Math.toRadians(2.0);

    // Table de calibration : {distance en pouces, vitesse shooter en TPS}
    private static final double[][] SHOOTER_TABLE = {
            {24.0, 1100},
            {36.0, 1200},
            {48.0, 1300},
            {60.0, 1400},
            {72.0, 1500},
            {84.0, 1600}
    };

    private double distance;
    private double angleToGoal;
    private double turnError;
    private double turnPower;
    private double targetTPS;
    private boolean aimed;

    public void update(double robotX, double robotY, double robotHeading) {
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;

        distance = Math.sqrt(dx * dx + dy * dy);

        angleToGoal = Math.atan2(dy, dx);
        turnError = normalizeAngle(angleToGoal - robotHeading);

        turnPower = clip(turnError * KP_TURN, -MAX_TURN_POWER, MAX_TURN_POWER);
        targetTPS = getShooterTPS(distance);

        aimed = Math.abs(turnError) < AIM_TOLERANCE_RAD;
    }

    public double getTurnPower() {
        return turnPower;
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

    private double getShooterTPS(double distance) {
        for (int i = 0; i < SHOOTER_TABLE.length - 1; i++) {
            double d1 = SHOOTER_TABLE[i][0];
            double tps1 = SHOOTER_TABLE[i][1];

            double d2 = SHOOTER_TABLE[i + 1][0];
            double tps2 = SHOOTER_TABLE[i + 1][1];

            if (distance >= d1 && distance <= d2) {
                double ratio = (distance - d1) / (d2 - d1);
                return tps1 + ratio * (tps2 - tps1);
            }
        }

        if (distance < SHOOTER_TABLE[0][0]) {
            return SHOOTER_TABLE[0][1];
        }

        return SHOOTER_TABLE[SHOOTER_TABLE.length - 1][1];
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}