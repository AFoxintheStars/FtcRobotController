package mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Shooter {
    // Left shooter
    private DcMotorEx flywheelLeft;
    private Servo angleServoLeft;

    //Right shooter

    private DcMotorEx flywheelRight;
    private Servo angleServoRight;

    // Constants
    private static final double TICKS_PER_REV = 28.0;

    // PIDF coefficients
    // F (feedforward) is critical for voltage-independent velocity
    private static final double SHOOTER_P = 10.0;
    private static final double SHOOTER_I = 0.5;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F = 12.0; // Key for consistent RPM across battery levels

    private static final double MAX_RPM = 5500.0; // Safe limit
    private static final double IDLE_RPM = 0.0;

    // === DISTANCE TO HOOD ANGLE SERVO POSITION (UNCHANGED) ===
    // Distance (inches) → servo pos 0-1 (higher = steeper angle)
    private static final double[][] DISTANCE_TO_ANGLE_TABLE = {
            { 12.0, 0.10 },   // Close: lower angle
            { 24.0, 0.11 },
            { 36.0, 0.12 },
            { 48.0, 0.13 },
            { 60.0, 0.14 },   // Farther: higher angle
            { 72.0, 0.15 }
    };

    // === DISTANCE TO TARGET RPM (TUNE EMPIRICALLY!) ===
    // Higher distance usually needs higher RPM, but higher angle reduces required RPM slightly
    private static final double[][] DISTANCE_TO_RPM_TABLE = {
            { 12.0, 2200.0 },  // Close: lower speed
            { 24.0, 2800.0 },
            { 36.0, 3400.0 },
            { 48.0, 3900.0 },
            { 60.0, 4400.0 },
            { 72.0, 4800.0 }   // Far: higher speed
    };

    private double targetRPMLeft = IDLE_RPM;
    private double targetRPMRight = IDLE_RPM;
    private boolean enabled = false;

    public void init(HardwareMap hwMap) {
        flywheelLeft = hwMap.get(DcMotorEx.class, "shooter_left");
        angleServoLeft = hwMap.get(Servo.class, "shooter_left_angle_servo");

        flywheelRight = hwMap.get(DcMotorEx.class, "shooter_right");
        angleServoRight = hwMap.get(Servo.class, "shooter_right_angle_servo");

        setupFlywheel(flywheelLeft);
        setupFlywheel(flywheelRight);

        // Starting Servo Position
        angleServoLeft.setPosition(0);
        angleServoRight.setPosition(1.0);
    }

    private void setupFlywheel(DcMotorEx flywheel) {
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set built-in PIDF for velocity control
        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    /**
     * Set target RPM directly
     */
    public void setTargetRPM(double rpm) {
        targetRPMLeft = targetRPMRight = Math.max(0, Math.min(rpm, MAX_RPM));
        enabled = targetRPMLeft > 0;
    }

    public void aimFromDistance(double distanceInches) {
        double servoPos = interpolate(distanceInches, DISTANCE_TO_ANGLE_TABLE);
        angleServoRight.setPosition(1.0 - servoPos);
        angleServoLeft.setPosition(servoPos);

        double rpm = interpolate(distanceInches, DISTANCE_TO_RPM_TABLE);
        targetRPMLeft = targetRPMRight = rpm;
        enabled = true;
    }

    /**
     * Calculate & set angle servo from distance (inches) from AprilTag.
     */
    private double interpolate(double distance, double[][] table) {
        if (distance <= table[0][0]) return table[0][1];
        int len = table.length;
        if (distance >= table[len-1][0]) return table[len-1][1];

        for (int i = 0; i < len - 1; i++) {
            double d1 = table[i][0];
            double d2 = table [i+1][0];
            if (distance >= d1 && distance <= d2) {
                double v1 = table[i][1];
                double v2 = table[i+1][1];
                double t = (distance - d1) / (d2 - d1);
                return v1 + t * (v2 - v1);
            }
        }
        return table[0][1];
    }

//    public void setAngleFromDistance(double distanceInches) {
//        if (distanceInches <= DISTANCE_TO_ANGLE_TABLE[0][0]) {
//            angleServo.setPosition(DISTANCE_TO_ANGLE_TABLE[0][1]);
//            return;
//        }
//        int len = DISTANCE_TO_ANGLE_TABLE.length;
//        if (distanceInches >= DISTANCE_TO_ANGLE_TABLE[len - 1][0]) {
//            angleServo.setPosition(DISTANCE_TO_ANGLE_TABLE[len - 1][1]);
//            return;
//        }
//
//        // Linear interpolation
//        for (int i = 0; i < len - 1; i++) {
//            double d1 = DISTANCE_TO_ANGLE_TABLE[i][0];
//            double d2 = DISTANCE_TO_ANGLE_TABLE[i + 1][0];
//            if (distanceInches >= d1 && distanceInches <= d2) {
//                double a1 = DISTANCE_TO_ANGLE_TABLE[i][1];
//                double a2 = DISTANCE_TO_ANGLE_TABLE[i + 1][1];
//                double t = (distanceInches - d1) / (d2 - d1);
//                angleServo.setPosition(a1 + t * (a2 - a1));
//                return;
//            }
//        }
//    }

    /**
     * Update flywheel velocity in loop.
     */
    public void update() {
        double ticksPerSec = rpmToTicksPerSec(targetRPMLeft);
        if (enabled) {
            flywheelLeft.setVelocity(ticksPerSec);
            flywheelRight.setVelocity(ticksPerSec);
        } else {
            flywheelLeft.setVelocity(0);
            flywheelRight.setVelocity(0);
        }
    }

    public double getCurrentRPMLeft() {
        return ticksPerSecToRPM(flywheelLeft.getVelocity());
    }

    public double getCurrentRPMRight() {
        return ticksPerSecToRPM(flywheelRight.getVelocity());
    }

    public boolean isAtTarget() {
        double tolerance = 100.0; // RPM tolerance
        return Math.abs(getCurrentRPMLeft() - targetRPMLeft) < tolerance &&
                Math.abs(getCurrentRPMRight() - targetRPMRight) < tolerance;
    }

    public void stop() {
        enabled = false;
        targetRPMLeft = targetRPMRight = IDLE_RPM;
        flywheelLeft.setVelocity(0);
        flywheelRight.setVelocity(0);
        angleServoLeft.setPosition(0);
        angleServoRight.setPosition(1.0);
    }

    // Physical angle estimation
    public double getEstimatedAngleDegrees() {
        double pos = angleServoLeft.getPosition();
        return 0.0 + pos * 80.0;
    }

    // Helpers
    private double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    private double ticksPerSecToRPM(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_REV) * 60.0;
    }

    // Telemetry helpers
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("Shooter Status:");
        telemetry.addData("Target RPM", "%.0f", targetRPMLeft);
        telemetry.addData("Left RPM", "%.0f", getCurrentRPMLeft());
        telemetry.addData("Right RPM", "%.0f", getCurrentRPMRight());
        telemetry.addData("At Target?", isAtTarget() ? "YES" : "NO");
        telemetry.addLine("Hood Servos:");
        telemetry.addData("Left Pos", "%.3f", angleServoLeft.getPosition());
        telemetry.addData("Right Pos", "%.3f", angleServoRight.getPosition());
        telemetry.addData("Est. Angle", "%.1f°", getEstimatedAngleDegrees());
    }
}