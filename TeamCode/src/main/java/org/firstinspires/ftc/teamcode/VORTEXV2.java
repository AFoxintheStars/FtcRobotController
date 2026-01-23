package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import mechanisms.Alliance;
import mechanisms.AprilTagAimer;
import mechanisms.AprilTagVision;
import mechanisms.Climber;
import mechanisms.Conveyor;
import mechanisms.DriveTrain;
import mechanisms.Intake;
import mechanisms.Shooter;
import mechanisms.Turntable;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "VORTEXV2")
@Disabled
public class VORTEXV2 extends OpMode {
    DriveTrain driveTrain = new DriveTrain();
    Intake intake = new Intake();
    Conveyor conveyor = new Conveyor();
    Turntable turntable = new Turntable();
    AprilTagVision aprilTagVision = new AprilTagVision();
    Shooter shooter = new Shooter();
    AprilTagAimer aprilTagAimer = new AprilTagAimer();
    Climber climber = new Climber();
    private boolean lastBState = false;
    private boolean lastAState = false;

    @Override
    public void init() {
        // int tagId = (getAlliance() == Alliance.BLUE) ? 20 : 24;

        driveTrain.init(hardwareMap);
        intake.init(hardwareMap);
        conveyor.init(hardwareMap);
        turntable.init(hardwareMap);
        // aprilTagVision.init(hardwareMap, tagId);
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

        // ===== CLIMBER =====
        climber.controlClimber(gamepad1.left_trigger, gamepad1.right_trigger);

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

        // ===== TURNTABLE MANUAL OVERRIDE =====
        turntable.checkManualNudge(gamepad2.left_bumper, gamepad2.right_bumper);

        // ===== APRIL TAG DETECTION =====

        // AprilTag processing & telemetry
        List<AprilTagDetection> detections = aprilTagVision.getAllianceDetections();

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
        // telemetry.addData("Detected Tags", aprilTagVision.getNumDetections());
        // telemetry.addData("Current Draw (A): ", intake.getCurrentDraw());
        telemetry.update();

    }

    @Override
    public void stop() {
        shooter.stop();
        aprilTagVision.close();
        super.stop();
    }
}