package mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagVision {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int targetTagId = -1;

    // Camera position relative to robot center (inches)
    // Example: Camera 6" forward, 4" left, 8" up from robot center
    private final Position cameraPosition = new Position(DistanceUnit.INCH,
            3,  // x (right/left; positive right)
            7.5,   // y (forward/back; positive forward)
            15.0,   // z (up/down; positive up)
            0);

    // Camera orientation (degrees)
    // Typical forward-facing horizontal camera: pitch = -90 (looking forward)
    // Yaw = 0 (forward), +90 left, -90 right, 180 backward
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0.0,   // yaw
            -90.0, // pitch (most common for forward horizontal)
            0.0,   // roll
            0);

    public void init(HardwareMap hwMap, int targetTagId) {
        this.targetTagId = targetTagId;

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)
                // .setLensIntrinsics(fx, fy, cx, cy) // only if needed for custom calibration
                .build();

        // Optional: Tune decimation for range vs speed trade-off
        //aprilTag.setDecimation(3); // default good balance

        VisionPortal.Builder builder = new VisionPortal.Builder()
        .setCamera(hwMap.get(WebcamName.class, "Webcam 1"))
        .enableLiveView(true)
        .addProcessor(aprilTag);

        visionPortal = builder.build();
    }

    /**
     * Get all current AprilTag detections.
     */
    public List<AprilTagDetection> getAllianceDetections() {
        List<AprilTagDetection> filtered = new ArrayList<>();
        for (AprilTagDetection d : aprilTag.getDetections()) {
            if (d.id == targetTagId) {
                filtered.add(d);
            }
        }
        return filtered;
    }

    /**
     * Get number of detected tags.
     */
    public int getNumDetections() {
        return getAllianceDetections().size();
    }

    /**
     * Close vision portal when OpMode ends.
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}