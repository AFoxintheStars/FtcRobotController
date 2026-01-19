package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import mechanisms.AprilTagVision;
import mechanisms.Conveyor;
import mechanisms.DriveTrain;
import mechanisms.Intake;
import mechanisms.Shooter;
import mechanisms.Turntable;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "VORTEXV2")
public class VORTEXV2 extends OpMode {
    DriveTrain driveTrain = new DriveTrain();
    Intake intake = new Intake();
    Conveyor conveyor = new Conveyor();
    Turntable turntable = new Turntable();
    AprilTagVision aprilTagVision = new AprilTagVision();
    Shooter shooter = new Shooter();

    @Override
    public void init() {
        driveTrain.init(hardwareMap);
        intake.init(hardwareMap);
        conveyor.init(hardwareMap);
        turntable.init(hardwareMap);
        aprilTagVision.init(hardwareMap);
        shooter.init(hardwareMap);

        telemetry.addLine("Vortex Initialized!");
    }

    @Override
    public void loop() {
        // ===== DRIVETRAIN =====
        double forward = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        driveTrain.teleopDrive(forward, turn);

        // ===== INTAKE =====
        intake.controlIntake(gamepad1.right_bumper, gamepad1.left_bumper);

        // ===== CONVEYOR =====
        if (gamepad1.y) {
            conveyor.forward();
        } else if (gamepad1.a) {
            conveyor.reverse();
        } else {
            conveyor.stop();
        }

        // ===== TURNTABLE PRESETS =====
        if (gamepad2.x) {
            turntable.goToAngle(0);
        } else if (gamepad2.y) {
            turntable.goToAngle(90);
        } else if (gamepad2.b) {
            turntable.goToAngle(180);
        } else if (gamepad2.a) {
            turntable.goToAngle(270);
        }

        // ===== APRIL TAG DETECTION =====

        // AprilTag processing & telemetry
        List<AprilTagDetection> detections = aprilTagVision.getDetections();

        telemetry.addData("Detected Tags", aprilTagVision.getNumDetections());

        // ===== SHOOTER & SERVO =====
        double distanceMeters = 0;
        boolean hasTarget = false;

        for (AprilTagDetection detection : aprilTagVision.getDetections()) {
            if (detection.metadata != null && detection.ftcPose != null) {
                distanceMeters = detection.ftcPose.range;
                hasTarget = true;
                break;
            }
        }

        if (gamepad1.x && hasTarget) {
            shooter.aimFromDistanceMeters(distanceMeters);
        } else {
            shooter.stop();
        }

        // ===== TELEMETRY =====

        turntable.update();
        shooter.update();

        shooter.addTelemetry(telemetry);
        telemetry.addData("Distance in", "%.2f", distanceMeters);
        telemetry.addData("Angle", turntable.getCurrentAngle());
        telemetry.addData("Target", turntable.getTargetAngle());
        telemetry.addData("At Target", turntable.atTarget());
        telemetry.addData("Detected Tags", aprilTagVision.getNumDetections());
        telemetry.update();

    }

    @Override
    public void stop() {
        shooter.stop();
        aprilTagVision.close();
        super.stop();
    }
}