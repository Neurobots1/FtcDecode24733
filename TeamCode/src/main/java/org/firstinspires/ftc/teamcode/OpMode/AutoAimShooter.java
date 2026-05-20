package org.firstinspires.ftc.teamcode.OpMode;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class AutoAimShooter {

    public static double GOAL_X = 3;
    public static double GOAL_Y = 141;

    public static double HEADING_OFFSET_DEG = -90;
    public static double TURN_DIRECTION = 1.0;

    public static double CHASSIS_AIM_KP = 0.12;
    public static double MAX_TURN_POWER = 0.16;
    public static double AIM_TOLERANCE_DEG = 7.0;
    public static double TURN_DEADBAND_DEG = 2.0;

    public static double SHOOTING_ZONE_RADIUS_INCHES = 10.0;

    public static double FLYWHEEL_ENCODER_TICKS_PER_REV = 28.0;

    private static final double[][] DISTANCE_RPM_TABLE = {
            {58.0, 1100.0},
            {66.0, 1125.0},
            {88.0, 1120.0},
            {101.0, 1313.0},
            {105.0, 1350.0}
    };

    private double distance;
    private double angleToGoal;
    private double turnError;
    private double turnPower;
    private double targetRPM;
    private double targetTPS;
    private boolean aimed;
    private boolean inSpinUpZone;

    public void update(double robotX, double robotY, double robotHeading) {
        double dx = GOAL_X - robotX;
        double dy = GOAL_Y - robotY;

        distance = Math.sqrt(dx * dx + dy * dy);

        angleToGoal = Math.atan2(dy, dx);

        double correctedRobotHeading = robotHeading + Math.toRadians(HEADING_OFFSET_DEG);

        turnError = normalizeAngle(angleToGoal - correctedRobotHeading);

        inSpinUpZone = isInShootingZone(robotX, robotY);

        updateHeadingLock();

        targetRPM = getShooterRPM(distance);
        targetTPS = targetRPM;

        aimed = Math.abs(turnError) < Math.toRadians(AIM_TOLERANCE_DEG);
    }

    public void resetHeadingLock() {
        turnPower = 0.0;
        aimed = false;
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

    public double getAngleToGoal() {
        return angleToGoal;
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

    private void updateHeadingLock() {
        double deadbandRad = Math.toRadians(TURN_DEADBAND_DEG);

        if (Math.abs(turnError) < deadbandRad) {
            turnPower = 0.0;
            return;
        }

        double command = turnError * CHASSIS_AIM_KP;

        command = clip(command, -MAX_TURN_POWER, MAX_TURN_POWER);

        turnPower = command * TURN_DIRECTION;
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

    private boolean isInShootingZone(double x, double y) {
        return isInBackZone(x, y, SHOOTING_ZONE_RADIUS_INCHES)
                || isInFrontZone(x, y, SHOOTING_ZONE_RADIUS_INCHES);
    }

    private boolean isInBackZone(double x, double y, double radiusInches) {
        double d = radiusInches * Math.sqrt(2);
        return y <= x - 48 + d && y <= -x + 96 + d;
    }

    private boolean isInFrontZone(double x, double y, double radiusInches) {
        double d = radiusInches * Math.sqrt(2);
        return y >= -x + 144 - d && y >= x - d;
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