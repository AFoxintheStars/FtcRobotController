package mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turntable_old {

    private Servo servo;
    private AnalogInput feedback;

    // ===============================
    // HARDWARE CONSTANTS
    // ===============================
    private static final double MAX_VOLTAGE = 3.3;
    private static final double TOTAL_DEGREES = 1800.0;

    private static final double ANGLE_TOLERANCE = 2.0;

    private static final double DEADBAND = 1.0;
    // PID tuning
    private static final double KP = 0.003;   // Less oscillation
    private static final double KI = 0.0005;   // Very small integral
    private static final double KD = 0.0005;   // Damping to stop stutter
    private static final double INTEGRAL_LIMIT = 0.2; // Anti-windup

    // Servo tuning
    private static final double SERVO_CENTER = 0.5;

    // ===== POSITIONS =====
    private static final double[] SHOOTING_ANGLES = {40.0, 125.0, 205.0};
    private static final double[] INTAKE_ANGLES = {0.0, 80.0, 165.0, 250.0};

    private double targetAngle = 0.0;

    // Cycle indices
    private int shootingIndex = 0;
    private int intakeIndex = 0;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    // Feedback filtering
    private double filteredAngle = 0.0;
    private static final double FILTER_ALPHA = 0.85;  // Higher = more smoothing

    // Manual override constants
    private static final double NUDGE_STEP_DEG = 5;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    public void init(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "turntable_servo");
        feedback = hwMap.get(AnalogInput.class, "turntable_feedback");

        servo.setPosition(SERVO_CENTER);
        targetAngle = 0.0;
        filteredAngle = getRawAngle();
        lastTime = System.currentTimeMillis() / 1000.0;
    }

    // ===== PUBLIC API =====

    /**
     * Press A: Cycle to next shooting position
     */
    public void cycleToNextShooting() {
        shootingIndex = (shootingIndex + 1) % SHOOTING_ANGLES.length;
        targetAngle = SHOOTING_ANGLES[shootingIndex];
    }

    /**
     * Press B: Cycle to next intake position
     * (No smart/closest logic — just next every time)
     */
    public void cycleToNextIntake() {
        intakeIndex = (intakeIndex + 1) % INTAKE_ANGLES.length;
        targetAngle = INTAKE_ANGLES[intakeIndex];
    }

    public void checkManualNudge(boolean leftBumperPressed, boolean rightBumperPressed) {
        if (leftBumperPressed && !lastLeftBumper) {
            targetAngle = normalize(targetAngle - NUDGE_STEP_DEG);
        }

        if (rightBumperPressed && !lastRightBumper) {
            targetAngle = normalize(targetAngle + NUDGE_STEP_DEG);
        }

        lastLeftBumper = leftBumperPressed;
        lastRightBumper = rightBumperPressed;
    }

    public double getCurrentAngle() {
        return normalize(filteredAngle);
    }

    public boolean atTarget() {
        return Math.abs(angleError(targetAngle, getCurrentAngle())) <= ANGLE_TOLERANCE;
    }

    public void update() {
        double raw = getRawAngle();
        filteredAngle = FILTER_ALPHA * filteredAngle + (1 - FILTER_ALPHA) * raw;

        double current = getCurrentAngle();
        double error = angleError(targetAngle, current);

        if (Math.abs(error) <= ANGLE_TOLERANCE) {
            servo.setPosition(servo.getPosition());
            integral = 0.0;
            return;
        }

        if (Math.abs(error) <= DEADBAND) {
            return;
        }

        // PID
        double now = System.currentTimeMillis() / 1000.0;
        double dt = now - lastTime;
        if (dt < 0.001) dt = 0.001;

        integral += error * dt;
        integral = clamp(integral, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

        double derivative = (error - lastError) / dt;

        double output = KP * error + KI * integral + KD * derivative;

        // Small power ramp to prevent twitch
        double maxPower = 0.6; // limit max servo speed
        output = Math.max(-maxPower, Math.min(maxPower, output));

        double servoCmd = SERVO_CENTER + output;
        servo.setPosition(clamp(servoCmd, 0.0, 1.0));

        lastError = error;
        lastTime = now;
    }

    private double getRawAngle() {
        return (feedback.getVoltage() / MAX_VOLTAGE) * TOTAL_DEGREES;
    }

    private double normalize(double angle) {
        angle %= TOTAL_DEGREES;
        if (angle < 0) angle += TOTAL_DEGREES;
        return angle;
    }

    private double angleError(double target, double current) {
        double error = target - current;
        error = (error + TOTAL_DEGREES/2) % TOTAL_DEGREES - TOTAL_DEGREES/2;
        return error;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getTargetAngle() {
        return targetAngle;
    }
}