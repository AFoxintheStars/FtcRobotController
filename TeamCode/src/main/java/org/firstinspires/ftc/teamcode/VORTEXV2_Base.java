package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

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

    private boolean singleDriverMode = false;
    private boolean lastStart = false;

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
        // SINGLE DRIVER MODE
        boolean startPressed = gamepad1.start;

        if (startPressed && !lastStart) {
            singleDriverMode = !singleDriverMode;
        }

        lastStart = startPressed;

        Gamepad operator = singleDriverMode ? gamepad1 : gamepad2;

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
        if (turntable.isMoving()) {
            conveyor.reverse();
        }
        else if (operator.dpad_up) {
            conveyor.forward();
        }
        else if (operator.dpad_down) {
            conveyor.reverse();
        }
        else {
            conveyor.stop();
        }

        // ===== CLIMBER =====
        climber.controlClimber(
                gamepad1.left_trigger,
                gamepad1.right_trigger,
                gamepad1.back);

        // ===== TURNTABLE =====
        turntable.handleManualControl(operator.dpad_left, operator.dpad_right);

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
        if (gamepad1.y) {
            if (hasTarget) {
                shooter.aimFromDistanceInches(distance);
            } else {
                shooter.aimDefault();  // fallback mode
            }
        } else {
            shooter.stop();
        }

        shooter.update();

        // ===== TELEMETRY =====
        shooter.addTelemetry(telemetry);
        telemetry.addData("Alliance", getAlliance());
        telemetry.addData("AprilTags Seen", detections.size());
        // telemetry.addData("Distance (in)", "%.1f", distance);
        telemetry.update();
    }

    @Override
    public void stop() {
        shooter.stop();
        aprilTagVision.close();
    }
}
