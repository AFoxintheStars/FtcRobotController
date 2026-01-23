package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import mechanisms.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public abstract class VORTEXV2_Base extends OpMode {

    // ===== ALLIANCE CONFIG =====
    protected abstract Alliance getAlliance();

    protected DriveTrain driveTrain = new DriveTrain();
    protected Intake intake = new Intake();
    protected Conveyor conveyor = new Conveyor();
    protected Turntable turntable = new Turntable();
    protected AprilTagVision aprilTagVision = new AprilTagVision();
    protected Shooter shooter = new Shooter();
    protected AprilTagAimer aprilTagAimer = new AprilTagAimer();
    protected Climber climber = new Climber();

    private boolean lastA = false;
    private boolean lastB = false;

    @Override
    public void init() {
        int tagId = (getAlliance() == Alliance.BLUE) ? 20 : 24;

        driveTrain.init(hardwareMap);
        intake.init(hardwareMap);
        conveyor.init(hardwareMap);
        turntable.init(hardwareMap);
        aprilTagVision.init(hardwareMap, tagId);
        shooter.init(hardwareMap);
        aprilTagAimer.init(hardwareMap, telemetry, driveTrain, aprilTagVision);
        climber.init(hardwareMap);

        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("Target AprilTag", tagId);
        telemetry.addLine("VORTEX READY");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ===== DRIVETRAIN =====
        double forward = gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        // ===== AIMING =====
        aprilTagAimer.setAimingEnabled(gamepad1.y);
        aprilTagAimer.update();

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

        // ===== CLIMBER =====
        climber.controlClimber(gamepad1.left_trigger, gamepad1.right_trigger);

        // ===== TURNTABLE =====
        if (gamepad2.a && !lastA) {
            turntable.cycleToNextShooting();
        }
        if (gamepad2.b && !lastB) {
            turntable.cycleToNextIntake();
        }
        lastA = gamepad2.a;
        lastB = gamepad2.b;

        turntable.checkManualNudge(gamepad2.left_bumper, gamepad2.right_bumper);
        turntable.update();

        // ===== APRILTAG DISTANCE =====
        double distance = 0;
        boolean hasTarget = false;

        List<AprilTagDetection> detections = aprilTagVision.getAllianceDetections();
        if (!detections.isEmpty()) {
            AprilTagDetection d = detections.get(0);
            if (d.ftcPose != null) {
                distance = d.ftcPose.range;
                hasTarget = true;
            }
        }

        // ===== SHOOTER =====
        if (gamepad1.y && hasTarget) {
            shooter.aimFromDistanceInches(distance);
        } else {
            shooter.stop();
        }

        shooter.update();

        // ===== TELEMETRY =====
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("AprilTags Seen", detections.size());
        telemetry.addData("Distance (in)", "%.1f", distance);
        telemetry.addData("Turntable Angle", "%.1f", turntable.getCurrentAngle());
        telemetry.addData("Turntable Target", "%.1f", turntable.getTargetAngle());
        telemetry.addData("Turntable At Target", turntable.atTarget());
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        aprilTagVision.close();
    }
}
