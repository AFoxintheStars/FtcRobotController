package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import mechanisms.AprilTagVision;
import mechanisms.Conveyor;
import mechanisms.DriveTrain;
import mechanisms.Intake;
import mechanisms.Shooter;
import mechanisms.Turntable;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
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

//        for (AprilTagDetection detection : detections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== Tag ID %d (%s)", detection.id, detection.metadata.name));
//
//                // Tag relative to camera (ftcPose)
//                if (detection.ftcPose != null) {
//                    telemetry.addLine(String.format("Tag XYZ (cam): %6.1f %6.1f %6.1f in",
//                            detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                    telemetry.addLine(String.format("Tag PRY: %6.1f %6.1f %6.1f deg",
//                            detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                    telemetry.addLine(String.format("Range/Bearing/Elev: %6.1f in, %6.1f°, %6.1f°",
//                            detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
//                }
//
//                // Robot global pose relative to field origin (requires setCameraPose)
//                if (detection.robotPose != null) {
//                    telemetry.addLine(String.format("Robot XYZ (field): %6.1f %6.1f %6.1f in",
//                            detection.robotPose.getPosition().x,
//                            detection.robotPose.getPosition().y,
//                            detection.robotPose.getPosition().z));
//                    telemetry.addLine(String.format("Robot Yaw/Pitch/Roll: %6.1f %6.1f %6.1f deg",
//                            detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                            detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES)));
//                }
//            } else {
//                telemetry.addLine(String.format("\n==== Unknown Tag ID %d", detection.id));
//            }
//        }

        // ===== SHOOTER & SERVO =====

        double distanceInches = 0;
        boolean hasTarget = false;

        for (AprilTagDetection detection : aprilTagVision.getDetections()) {
            if (detection.metadata != null && detection.ftcPose != null) {
                distanceInches = detection.ftcPose.range; // or use robotPose if needed
                hasTarget = true;
                break;
            }
        }

        if (gamepad1.x && hasTarget) {
            shooter.aimFromDistanceMeters(distanceInches);  // sets angle + RPM for both
        } else {
            shooter.stop();
        }

        Position robotPos = aprilTagVision.getRobotPosition();
        if (robotPos != null) {
            telemetry.addData("Robot Field Pos (first valid)", String.format("%.1f, %.1f, %.1f in",
                    robotPos.x, robotPos.y, robotPos.z));
        }

        // ===== TELEMETRY =====

        turntable.update();
        shooter.update();

        shooter.addTelemetry(telemetry);
        telemetry.addData("Conveyor Velocity", conveyor.getVelocity());
        telemetry.addData("Conveyor Target Velocity", conveyor.getTargetVelocity());
        telemetry.addData("Intake Current (A)", intake.getCurrentDraw());
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