package mechanisms;

public final class ShooterMath {

    // ================== PHYSICAL CONSTANTS ==================
    public static final double GRAVITY = 9.81;

    public static final double SHOOTER_HEIGHT_M = 0.35;
    public static final double TARGET_HEIGHT_M = 1.20;

    // RPM to exit velocity conversion
    // This is your biggest tuning constant
    // Typical range for hooded shooters with 3–5" wheels: 0.0006 to 0.0012 m/s per RPM
    // Start with ~0.00085 and tune by shooting at known distances
    public static final double RPM_TO_VELOCITY_COEFF = 0.00085;

    public static final double SERVO_MIN_POSITION = 0.00;
    public static final double SERVO_MAX_POSITION = 0.05;
    public static final double SERVO_MIN_ANGLE_DEG = 0.0;
    public static final double SERVO_MAX_ANGLE_DEG = 80.0;

    private ShooterMath() {}

    /**
     * Convert flywheel RPM → muzzle velocity (m/s)
     */
    public static double rpmToVelocity(double rpm) {
        return rpm * RPM_TO_VELOCITY_COEFF;
    }

    public static double velocityToRpm(double velocityMps) {
        return velocityMps / RPM_TO_VELOCITY_COEFF;
    }

    /**
     * Horizontal distance from robot to target (m)
     */
    public static double computeRangeMeters(double fieldXMeters, double fieldYMeters) {
        return Math.hypot(fieldXMeters, fieldYMeters);
    }

    /**
     * Vertical error for a given launch angle (radians)
     * Positive error = overshoots height, negative = undershoots
     */
    private static double verticalError(
            double rangeMeters,
            double velocityMps,
            double launchAngleRad,
            double deltaZMeters) {

        if (Math.abs(Math.cos(launchAngleRad)) < 1e-6) {
            return Double.POSITIVE_INFINITY;
        }

        double term1 = rangeMeters * Math.tan(launchAngleRad);
        double term2 = (GRAVITY * rangeMeters * rangeMeters)
                / (2.0 * velocityMps * velocityMps * Math.pow(Math.cos(launchAngleRad), 2));

        return term1 - term2 - deltaZMeters;
    }

    /**
     * Solve for required muzzle velocity (m/s) given fixed launch angle
     * @param rangeMeters     horizontal distance
     * @param launchAngleRad  desired fixed launch angle
     * @param deltaZMeters    target height - shooter height
     * @return required velocity in m/s, or NaN if impossible
     */
    public static double solveVelocityMps(
            double rangeMeters,
            double launchAngleRad,
            double deltaZMeters) {

        if (rangeMeters < 0.1 || Math.abs(Math.cos(launchAngleRad)) < 1e-6) {
            return Double.NaN;
        }

        // Rearranged projectile equation for velocity
        double tanTheta = Math.tan(launchAngleRad);
        double cos2Theta = Math.pow(Math.cos(launchAngleRad), 2);

        double numerator = GRAVITY * rangeMeters * rangeMeters;
        double denominator = 2 * cos2Theta * (rangeMeters * tanTheta - deltaZMeters);

        if (denominator <= 0) {
            return Double.NaN; // impossible (undershoot even at infinite speed)
        }

        return Math.sqrt(numerator / denominator);
    }

    /**
     * Numerically solve for best launch angle (radians) using binary search / bisection
     * @param rangeMeters     horizontal distance
     * @param velocityMps     muzzle velocity
     * @param deltaZMeters    target height - shooter height
     * @return launch angle in radians (between ~10° and 80°)
     */
    public static double solveLaunchAngleRad(
            double rangeMeters,
            double velocityMps,
            double deltaZMeters) {

        if (rangeMeters < 0.1) {
            return Math.toRadians(45);
        }

        double low = Math.toRadians(10.0);
        double high = Math.toRadians(80.0);

        for (int i = 0; i < 35; i++) {
            double mid = (low + high) / 2.0;
            double error = verticalError(rangeMeters, velocityMps, mid, deltaZMeters);

            if (error > 0) {
                high = mid;  // too high → reduce angle
            } else {
                low = mid;   // too low → increase angle
            }
        }

        return (low + high) / 2.0;
    }

    /**
     * Convert solved launch angle (rad) → servo position [0.0 .. 1.0]
     */
    public static double angleRadToServoPosition(double angleRad) {
        double angleDeg = Math.toDegrees(angleRad);

        angleDeg = Math.max(SERVO_MIN_ANGLE_DEG, Math.min(SERVO_MAX_ANGLE_DEG, angleDeg));

        double position = (angleDeg - SERVO_MIN_ANGLE_DEG) / (SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG);

        position = Math.max(SERVO_MIN_POSITION, Math.min(SERVO_MAX_POSITION, position));

        return position;
    }
}