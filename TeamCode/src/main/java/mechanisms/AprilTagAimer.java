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
    private static final double KP = 0.05;   // Proportional - how aggressively it turns
    private static final double KI = 0.000;   // Integral - usually small or 0
    private static final double KD = 0.002;   // Derivative - dampens oscillation

    // Tolerance to consider "facing" the tag (± degrees)
    private static final double BEARING_TOLERANCE_DEG = 3.0;

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
     * Toggle aiming mode on/off
     */
    public void toggleAiming() {
        aimingActive = !aimingActive;
        if (!aimingActive) {
            driveTrain.teleopDrive(0, 0); // stop turning when disabled
            integral = 0.0;
        }
        telemetry.addData("Aimer Status", aimingActive ? "ACTIVE" : "OFF");
    }

    /**
     * Call this every loop when aiming is active
     */
    public void update() {
        if (!aimingActive) return;

        double bearing = getTagBearing();

        if (Double.isNaN(bearing)) {
            // No valid tag → stop turning, show warning
            driveTrain.teleopDrive(0, 0);
            telemetry.addData("Aimer", "No valid AprilTag visible");
            return;
        }

        // PID calculation
        double currentTime = System.currentTimeMillis() / 1000.0;
        double dt = currentTime - lastTime;
        if (dt < 0.001) dt = 0.001;

        double error = -bearing; // positive = turn right (adjust sign if robot turns wrong way)
        integral += error * dt;
        double derivative = (error - lastError) / dt;

        double turnPower = KP * error + KI * integral + KD * derivative;

        // Clamp to safe power
        turnPower = Math.max(-0.8, Math.min(0.8, turnPower)); // limit max turn speed

        // Apply turn (no forward movement)
        driveTrain.teleopDrive(0, turnPower);

        lastError = error;
        lastTime = currentTime;

        // Telemetry for debugging
        telemetry.addData("Aimer", "ACTIVE");
        telemetry.addData("Tag Bearing", "%.1f°", bearing);
        telemetry.addData("Turn Power", "%.2f", turnPower);
        telemetry.addData("Facing?", isFacingTag() ? "YES" : "Turning...");
    }

    /**
     * Is robot facing the tag within tolerance?
     */
    public boolean isFacingTag() {
        double bearing = getTagBearing();
        return !Double.isNaN(bearing) && Math.abs(bearing) <= BEARING_TOLERANCE_DEG;
    }

    // Get bearing to first valid tag (degrees), NaN if none
    private double getTagBearing() {
        List<AprilTagDetection> detections = aprilTagVision.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.ftcPose != null) {
                return detection.ftcPose.bearing;
            }
        }
        return Double.NaN;
    }
}