package mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AprilTagAimer {

    private DriveTrain driveTrain;
    private AprilTagVision aprilTagVision;
    private Telemetry telemetry;

    // PID constants - tune these!
    private static final double KP = 0.055;   // Proportional - how aggressively it turns
    private static final double KI = 0.000;
    private static final double KD = 0.002;   // Derivative
    private static final double BEARING_TOLERANCE_DEG = 1.0;

    // PID state
    private double integral = 0.0;
    private double lastError = 0.0;
    private double lastTime = 0.0;

    public boolean aimingActive = false;

    public void init(HardwareMap hwMap, Telemetry telemetry, DriveTrain driveTrain, AprilTagVision aprilTagVision) {
        this.driveTrain = driveTrain;
        this.aprilTagVision = aprilTagVision;
        this.telemetry = telemetry;
    }

    /**
     * Enable or disable aiming mode
     * @param enabled true = aim while held, false = stop aiming
     */
    public void setAimingEnabled(boolean enabled) {
        if (enabled && !aimingActive) {
            // Reset PID state when turning on
            integral = 0.0;
            lastError = 0.0;
            lastTime = System.currentTimeMillis() / 1000.0;
        }

        aimingActive = enabled;

        if (!enabled) {
            // Immediately stop turning when disabled
            driveTrain.teleopDrive(0, 0);
        }
    }

    /**
     * Call this every loop when aiming is active
     */
    public void update() {
        if (!aimingActive) return;

        double bearing = getTagBearing();

        if (Double.isNaN(bearing)) {
            driveTrain.teleopDrive(0, 0);
            telemetry.addData("Aimer", "No valid AprilTag visible");
            return;
        }

        // PID calculation
        double currentTime = System.currentTimeMillis() / 1000.0;
        double dt = currentTime - lastTime;
        if (dt < 0.001) dt = 0.001;

        double error = -bearing; // positive = turn right
        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double turnPower = KP * error + KI * integral + KD * derivative;

        turnPower = Math.max(-0.9, Math.min(0.9, turnPower)); // limit max turn speed

        driveTrain.teleopDrive(0, turnPower);

        lastError = error;
        lastTime = currentTime;

        // Telemetry
        telemetry.addData("Aimer", "ACTIVE");
        telemetry.addData("Turn Power", "%.2f", turnPower);
    }

    /**
     * Is robot facing the tag within tolerance?
     */
    public boolean isFacingTag() {
        double bearing = getTagBearing();
        return !Double.isNaN(bearing) && Math.abs(bearing) <= BEARING_TOLERANCE_DEG;
    }

    private double getTagBearing() {
        List<AprilTagDetection> detections = aprilTagVision.getAllianceDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.ftcPose != null) {
                return detection.ftcPose.bearing;
            }
        }
        return Double.NaN;
    }
}