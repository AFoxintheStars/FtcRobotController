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

    // Right shooter
    private DcMotorEx flywheelRight;
    private Servo angleServoRight;

    // Constants
    private static final double TICKS_PER_REV = 28.0;

    // PIDF coefficients
    private static final double SHOOTER_P = 10.0;
    private static final double SHOOTER_I = 0.5;
    private static final double SHOOTER_D = 0.0;
    private static final double SHOOTER_F = 12.0;

    private static final double MIN_RPM = 1900.0;
    private static final double MAX_RPM = 2200.0;
    private static final double IDLE_RPM = 0.0;

    private double targetRPMLeft = IDLE_RPM;
    private double targetRPMRight = IDLE_RPM;
    private boolean enabled = false;

    // === DISTANCE TO HOOD ANGLE SERVO POSITION (inches → position 0-1) ===
    // Higher distance → higher (steeper) hood angle
    private static final double[][] DISTANCE_TO_ANGLE_TABLE = {
            { 12.0, 0.00 },   // Very close: flattest
            { 24.0, 0.01 },
            { 36.0, 0.02 },
            { 48.0, 0.03 },
            { 60.0, 0.04 },
            { 72.0, 0.05 }    // Farthest: steepest (max safe)
    };

    // === DISTANCE TO TARGET RPM (inches → RPM) ===
    // Higher distance → higher RPM
    private static final double[][] DISTANCE_TO_RPM_TABLE = {
            { 12.0, 1900.0 }, // Close: lower speed
            { 24.0, 1950.0 },
            { 36.0, 2000.0 },
            { 48.0, 2050.0 },
            { 60.0, 2100.0 },
            { 72.0, 2200.0 }  // Far: max speed
    };

    public void init(HardwareMap hwMap) {
        flywheelLeft = hwMap.get(DcMotorEx.class, "shooter_left");
        angleServoLeft = hwMap.get(Servo.class, "shooter_left_angle_servo");

        flywheelRight = hwMap.get(DcMotorEx.class, "shooter_right");
        angleServoRight = hwMap.get(Servo.class, "shooter_right_angle_servo");

        setupFlywheel(flywheelLeft, true);  // reversed
        setupFlywheel(flywheelRight, false);

        // Starting Servo Position (lowest hood)
        angleServoLeft.setPosition(0.0);
        angleServoRight.setPosition(1.0);
    }

    private void setupFlywheel(DcMotorEx flywheel, boolean reversed) {
        flywheel.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
    }

    /**
     * Aim using distance tables (inches)
     */
    public void aimFromDistanceInches(double distanceInches) {
        // Hood position from table
        double servoPosLeft = interpolate(distanceInches, DISTANCE_TO_ANGLE_TABLE);

        angleServoLeft.setPosition(servoPosLeft);
        angleServoRight.setPosition(1.0 - servoPosLeft); // inverted

        // RPM from table
        double rpm = interpolate(distanceInches, DISTANCE_TO_RPM_TABLE);
        rpm = Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));

        targetRPMLeft = targetRPMRight = rpm;
        enabled = true;
    }

    // Linear interpolation helper (used for both tables)
    private double interpolate(double distance, double[][] table) {
        if (distance <= table[0][0]) return table[0][1];
        int len = table.length;
        if (distance >= table[len - 1][0]) return table[len - 1][1];

        for (int i = 0; i < len - 1; i++) {
            double d1 = table[i][0];
            double d2 = table[i + 1][0];
            if (distance >= d1 && distance <= d2) {
                double v1 = table[i][1];
                double v2 = table[i + 1][1];
                double t = (distance - d1) / (d2 - d1);
                return v1 + t * (v2 - v1);
            }
        }
        return table[0][1]; // fallback
    }

    /**
     * Update flywheel velocity every loop
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
        double tolerance = 100.0;
        return Math.abs(getCurrentRPMLeft() - targetRPMLeft) < tolerance &&
                Math.abs(getCurrentRPMRight() - targetRPMRight) < tolerance;
    }

    public void stop() {
        enabled = false;
        targetRPMLeft = targetRPMRight = IDLE_RPM;
        flywheelLeft.setVelocity(0);
        flywheelRight.setVelocity(0);
        angleServoLeft.setPosition(0.0);
        angleServoRight.setPosition(1.0);
    }

    // Physical angle estimation (for telemetry)
    public double getEstimatedAngleDegrees() {
        double pos = angleServoLeft.getPosition();
        return pos * 5.0; // since your max is ~5°
    }

    // Helpers
    private double rpmToTicksPerSec(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    private double ticksPerSecToRPM(double ticksPerSec) {
        return (ticksPerSec / TICKS_PER_REV) * 60.0;
    }

    // Telemetry
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addLine("Shooter Status:");
        telemetry.addData("Target RPM", "%.0f", targetRPMLeft);
        telemetry.addData("Left Current RPM", "%.0f", getCurrentRPMLeft());
        telemetry.addData("Right Current RPM", "%.0f", getCurrentRPMRight());
        telemetry.addData("At Target?", isAtTarget() ? "YES" : "NO");
        telemetry.addLine("Hood Servos:");
        telemetry.addData("Left Pos", "%.3f", angleServoLeft.getPosition());
        telemetry.addData("Right Pos", "%.3f", angleServoRight.getPosition());
        telemetry.addData("Est. Launch Angle", "%.1f°", getEstimatedAngleDegrees());
    }
}