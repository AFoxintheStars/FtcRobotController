package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import mechanisms.*;

import java.util.List;

public abstract class VORTEX_AutoBase extends LinearOpMode {

    protected abstract Alliance getAlliance();

    protected DriveTrain driveTrain;
    protected AprilTagVision aprilTagVision;
    protected AprilTagAimer aprilTagAimer;
    protected Shooter shooter;
    protected Turntable turntable;
    protected Conveyor conveyor;

    private static final int BALL_COUNT = 3;

    @Override
    public void runOpMode() {

        int tagId = (getAlliance() == Alliance.BLUE) ? 20 : 24;

        driveTrain = new DriveTrain();
        aprilTagVision = new AprilTagVision();
        aprilTagAimer = new AprilTagAimer();
        shooter = new Shooter();
        turntable = new Turntable();
        conveyor = new Conveyor();

        driveTrain.init(hardwareMap);
        aprilTagVision.init(hardwareMap, tagId);
        aprilTagAimer.init(hardwareMap, telemetry, driveTrain, aprilTagVision);
        shooter.init(hardwareMap);
        turntable.init(hardwareMap);
        conveyor.init(hardwareMap);

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Target Tag", tagId);
        telemetry.addLine("Auto Ready");
        telemetry.update();

        waitForStart();
        if (!opModeIsActive()) return;

        /* ================= AIM AT TAG ================= */
        ElapsedTime aimTimer = new ElapsedTime();
        aprilTagAimer.setAimingEnabled(true);

        while (opModeIsActive()
                && !aprilTagAimer.isFacingTag()
                && aimTimer.seconds() < 2.0) {

            aprilTagAimer.update();
            sleep(20);
        }

        aprilTagAimer.setAimingEnabled(false);

        /* ================= GET DISTANCE ================= */
        double distanceInches = 115.0; // fallback
        List<AprilTagDetection> detections = aprilTagVision.getAllianceDetections();
        if (!detections.isEmpty() && detections.get(0).ftcPose != null) {
            distanceInches = detections.get(0).ftcPose.range;
        }

        /* ================= PREP SHOOTER ================= */
        shooter.aimFromDistanceInches(distanceInches);

        ElapsedTime spinupTimer = new ElapsedTime();

        while (opModeIsActive()
                && !shooter.isReadyToFire()
                && spinupTimer.seconds() < 2.5) {

            shooter.addTelemetry(telemetry);
            shooter.update();

            telemetry.addData("Shooter RPM L", shooter.getCurrentRPMLeft());
            telemetry.addData("Shooter RPM R", shooter.getCurrentRPMRight());
            telemetry.update();

            sleep(20);
        }

        /* ================= FIRE BALLS ================= */
        for (int i = 0; i < BALL_COUNT && opModeIsActive(); i++) {

            // ---- Rotate turntable for 2 seconds (anti-jam) ----
            ElapsedTime timer = new ElapsedTime();
            turntable.setPower(0.3);
            conveyor.reverse(); // move balls DOWN

            while (opModeIsActive() && timer.seconds() < 0.7) {
                shooter.update();
                sleep(20);
            }

            turntable.stop();
            conveyor.stop();

            // ---- Feed ball UP ----
            conveyor.forward();
            sleep(2000);
            conveyor.stop();

            // ---- Ensure shooter still ready ----
            while (opModeIsActive() && !shooter.isReadyToFire()) {
                shooter.update();
                sleep(20);
            }
        }

        /* ================= SHUTDOWN ================= */
        shooter.stop();
        conveyor.stop();
        turntable.stop();

        /* ================= PARK ================= */
        driveTrain.teleopDrive(-0.5, 0);
        sleep(700);
        driveTrain.teleopDrive(0, 0);
    }
}
