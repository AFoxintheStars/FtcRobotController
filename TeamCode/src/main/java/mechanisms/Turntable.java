package mechanisms;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turntable {

    private Servo servo;
    private AnalogInput feedback;

    // ===============================
    // TURNABLE HARDWARE CONSTANTS
    // ===============================
    private static final double MAX_VOLTAGE = 3.3;
    private static final double TOTAL_DEGREES = 1800.0;

    private static final double ANGLE_TOLERANCE = 3.0;

    // Servo tuning
    private static final double SERVO_CENTER = 0.5;
    private static final double kP = 0.006; // servo units per degree

    private double targetAngle = 0.0;

    public void init(HardwareMap hwMap) {
        servo = hwMap.get(Servo.class, "turntable_servo");
        feedback = hwMap.get(AnalogInput.class, "turntable_feedback");

        servo.setPosition(SERVO_CENTER);
    }

    // ===== PUBLIC API =====

    public void goToAngle(double degrees) {
        targetAngle = normalize(degrees);
    }

    public double getCurrentAngle() {
        return normalize((feedback.getVoltage() / MAX_VOLTAGE) * TOTAL_DEGREES);
    }

    public boolean atTarget() {
        return Math.abs(angleError(targetAngle, getCurrentAngle())) <= ANGLE_TOLERANCE;
    }

    // Call every loop
    public void update() {
        double currentAngle = getCurrentAngle();
        double error = angleError(targetAngle, currentAngle);

        if (Math.abs(error) <= ANGLE_TOLERANCE) {
            servo.setPosition(servo.getPosition()); // hold
            return;
        }

        double servoCommand =
                SERVO_CENTER + (kP * error);

        servo.setPosition(clamp(servoCommand, 0.0, 1.0));
    }

    // ===== WRAP LOGIC =====

    private double normalize(double angle) {
        angle %= TOTAL_DEGREES;
        if (angle < 0) angle += TOTAL_DEGREES;
        return angle;
    }

    private double angleError(double target, double current) {
        double error = target - current;
        if (error > TOTAL_DEGREES / 2) error -= TOTAL_DEGREES;
        if (error < -TOTAL_DEGREES / 2) error += TOTAL_DEGREES;
        return error;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public double getTargetAngle() {
        return targetAngle;
    }
}
