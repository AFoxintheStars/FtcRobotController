package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import mechanisms.AprilTagAimer;
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
    AprilTagAimer aprilTagAimer = new AprilTagAimer();
    private boolean lastBState = false;
    private boolean lastAState = false;

    @Override
    public void init() {
        driveTrain.init(hardwareMap);
        intake.init(hardwareMap);
        conveyor.init(hardwareMap);
        turntable.init(hardwareMap);
        aprilTagVision.init(hardwareMap);
        shooter.init(hardwareMap);
        aprilTagAimer.init(hardwareMap, telemetry, driveTrain, aprilTagVision);

        telemetry.addLine("Vortex Initialized!");
    }

    @Override
    public void loop() {
        // ===== DRIVETRAIN =====
        double forward = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        driveTrain.teleopDrive(forward, turn);

        // ===== AIMING =====
        aprilTagAimer.setAimingEnabled(gamepad1.x);

        // ===== Aiming update =====
        aprilTagAimer.update();

        // Only allow manual driving when NOT aiming
        if (!aprilTagAimer.aimingActive) {
            driveTrain.teleopDrive(forward, turn);
        }

        // ===== INTAKE =====
        intake.controlIntake(gamepad1.right_bumper, gamepad1.left_bumper);

        // ===== CONVEYOR =====
        if (gamepad2.x) {
            conveyor.forward();
        } else if (gamepad2.y) {
            conveyor.reverse();
        } else {
            conveyor.stop();
        }

        // ===== TURNTABLE CONTROL =====

        if (gamepad2.a && !lastAState) {
            turntable.cycleToNextShooting();
        }
        lastAState = gamepad2.a;

        if (gamepad2.b && !lastBState) {
            turntable.cycleToNextIntake();
        }
        lastBState = gamepad2.b;

        // ===== TURNTABLE MANUAL OVERRIDE (D-pad nudge) =====
        if (gamepad2.dpad_right) {
            turntable.nudgeLeft();
        } else if (gamepad2.dpad_left) {
            turntable.nudgeRight();
        } else {
            turntable.clearNudge();
        }

        // ===== APRIL TAG DETECTION =====

        // AprilTag processing & telemetry
        List<AprilTagDetection> detections = aprilTagVision.getDetections();

        telemetry.addData("Detected Tags", aprilTagVision.getNumDetections());

        // ===== SHOOTER & SERVO =====
        double distanceInches = 0;
        boolean hasTarget = false;

        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null && detection.ftcPose != null) {
                distanceInches = detection.ftcPose.range;
                hasTarget = true;
                break;
            }
        }

        if (gamepad1.x && hasTarget) {
            shooter.aimFromDistanceInches(distanceInches);
        } else {
            shooter.stop();
        }

        // ===== TELEMETRY =====

        turntable.update();
        shooter.update();

        shooter.addTelemetry(telemetry);
        telemetry.addData("Aimer Active?", aprilTagAimer.aimingActive ? "YES (B pressed)" : "NO");
        if (aprilTagAimer.aimingActive && aprilTagAimer.isFacingTag()) {
            telemetry.addLine("→ Robot is facing the AprilTag!");
        }
        telemetry.addData("Distance in", "%.2f", distanceInches);
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