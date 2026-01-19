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
    private static final double SHOOTER_F = 12.0;
    private static final double DEFAULT_TARGET_RPM = 3800.0;
    private static final double MIN_RPM = 1800.0;
    private static final double MAX_RPM = 5200.0;
    private static final double IDLE_RPM = 0.0;
    public double desiredRPM = DEFAULT_TARGET_RPM;
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

    public void setDesiredRPM(double rpm) {
        desiredRPM = Math.max(MIN_RPM, Math.min(MAX_RPM, rpm));
    }

    /**
     * Aim both hoods and set flywheels using physics solver
     * @param distanceMeters horizontal distance to target (from AprilTag)
     */
    public void aimFromDistanceMeters(double distanceMeters) {
        double velocity = ShooterMath.rpmToVelocity(desiredRPM);
        double deltaZ = ShooterMath.TARGET_HEIGHT_M - ShooterMath.SHOOTER_HEIGHT_M;

        double angleRad = ShooterMath.solveLaunchAngleRad(distanceMeters, velocity, deltaZ);
        double servoPosLeft = ShooterMath.angleRadToServoPosition(angleRad);

        angleServoLeft.setPosition(servoPosLeft);
        angleServoRight.setPosition(1.0 - servoPosLeft); // inverted

        targetRPMLeft = targetRPMRight = desiredRPM;
        enabled = true;
    }

    public void aimFromDistanceInches(double distanceInches) {
        aimFromDistanceMeters(distanceInches / 39.3701); // inches → meters
    }

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
        telemetry.addData("Target RPM", "%.0f", desiredRPM);
        telemetry.addData("Left RPM", "%.0f", getCurrentRPMLeft());
        telemetry.addData("Right RPM", "%.0f", getCurrentRPMRight());
        telemetry.addData("At Target?", isAtTarget() ? "YES" : "NO");
        telemetry.addLine("Hood Servos:");
        telemetry.addData("Left Pos", "%.3f", angleServoLeft.getPosition());
        telemetry.addData("Right Pos", "%.3f", angleServoRight.getPosition());
        telemetry.addData("Est. Launch Angle", "%.1f°", getEstimatedAngleDegrees());
    }
}