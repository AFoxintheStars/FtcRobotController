/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Vortex DECODE", group="TeleOp")
@Disabled
public class VORTEXV1 extends LinearOpMode {

    // Hardware
    private DcMotor shooterLeft;
    private DcMotor shooterRight;
    private DcMotor conveyorMotor;
    private DcMotor driveMotorLeft;
    private DcMotor driveMotorRight;
    private DcMotor intakeMotor;
    private DcMotor liftMotorRight;
    private DcMotor liftMotorLeft;
    private CRServo turnTableServo;

    // State Variables
    private String shootState = "IDLE";
    // private String lastShootState = "";
    private boolean shootingActive = false;
    private boolean lastIntakeForward = false;
    private boolean lastIntakeReverse = false;
    private boolean lastConveyorForward = false;
    private boolean lastConveyorReverse = false;
    private boolean lastCycleButton = false;
    private boolean tankDriveMode = false;
    private boolean lastStartButton = false;

    // Timers
    private final ElapsedTime sectorTimer = new ElapsedTime();
    private final ElapsedTime shooterTimer = new ElapsedTime();

    // Turntable Tracking
    private int turnTableSector = 0;
    private double currentPosition = 0.0;
    private final double sectorsPerCircle = 3.0;
    private final double sectorOffsetForFeed = 1.5;
    private final int sectorTime = 535;
    private boolean isTurning = false;

    // Lift Settings
    private final double liftPowerUp = 1.0;
    private final double liftPowerDown = 0.8;
    private final double levelCorrectionGain = 0.02;
    private final int levelToleranceTicks = 20;

    // Drive Settings
    private final double maxDriveSpeed = 1.0;

    @Override
    public void runOpMode() {
        initializeHardware();

        telemetry.addData("Status", "Initialized - Ready to Start");
        telemetry.update();

        shootingActive = false;
        waitForStart();
        if (opModeIsActive()) {
            while (opModeIsActive()) {
                conveyorSubsystem();
                driveSubsystem();
                intakeSubsystem();
                shooterSubsystem();
                turnTableSubsystem();
                liftSubsystem();
                telemetry.update();
            }
        }
    }

    private void initializeHardware() {
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        driveMotorLeft = hardwareMap.get(DcMotor.class, "driveMotorLeft");
        driveMotorRight = hardwareMap.get(DcMotor.class, "driveMotorRight");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        liftMotorLeft = hardwareMap.get(DcMotor.class, "liftMotorLeft");
        liftMotorRight = hardwareMap.get(DcMotor.class, "liftMotorRight");
        turnTableServo = hardwareMap.get(CRServo.class, "turnTableServo");


        // Directions
        driveMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        driveMotorRight.setDirection(DcMotor.Direction.FORWARD);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        conveyorMotor.setDirection(DcMotor.Direction.FORWARD);
        liftMotorRight.setDirection(DcMotor.Direction.REVERSE);
        liftMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        turnTableServo.setDirection(CRServo.Direction.FORWARD);

        // Zero Power Behavior (Brake)
        driveMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        conveyorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset Encoders
        liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Controls robot movement using arcade drive.
     * Left stick Y = forward/back,
     * Left stick X = turn.
     */
    private void driveSubsystem() {
        double forward;
        double turn;
        double leftStickY;
        double rightStickY;
        double rightPower;
        double leftPower;
        double speedMultiplier;
        double triggerValue;

        boolean currentStartButton = gamepad1.start;
        if (currentStartButton && !lastStartButton) {
            tankDriveMode = !tankDriveMode;
        }
        lastStartButton = currentStartButton;

        triggerValue = gamepad1.left_trigger;
        speedMultiplier = triggerValue > 0.1
                ? maxDriveSpeed * (1.0 - triggerValue)
                : maxDriveSpeed;

        if (tankDriveMode) {
            leftStickY = gamepad1.left_stick_y;
            rightStickY = gamepad1.right_stick_y;

            leftPower = leftStickY * speedMultiplier;
            rightPower = rightStickY * speedMultiplier;

            telemetry.addData("Drive Type", "TANK");
        } else {
            forward = gamepad1.left_stick_y;
            turn = -gamepad1.right_stick_x;

            rightPower = (forward + turn) * speedMultiplier;
            leftPower = (forward - turn) * speedMultiplier;

            telemetry.addData("Drive Type", "ARCADE");
        }

        driveMotorLeft.setPower(Range.clip(leftPower, -1.0, 1.0));
        driveMotorRight.setPower(Range.clip(rightPower, -1.0, 1.0));

        telemetry.addData("Drive Mode", gamepad1.left_bumper ? "SLOW" : "NORMAL");
        telemetry.addData("Speed Limit", "%.0f%%", speedMultiplier * 100);
        telemetry.addData("Left Power", "%.2f", leftPower);
        telemetry.addData("Right Power", "%.2f", rightPower);
    }

    private void setShooterPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    /**
     * Runs both manual and automatic shooter control.
     * Manual control uses gamepad1 triggers.
     * Auto macro runs a timed spinup → feed → cooldown sequence.
     */
    private void shooterSubsystem() {
        double rightTriggerValue;
        double leftTriggerValue;

        // Manual Shooter Control
        if (shootState.equals("IDLE")) {
            rightTriggerValue = gamepad2.right_trigger;
            leftTriggerValue = gamepad2.left_trigger;
            if (rightTriggerValue > 0.0) {
                setShooterPower(-rightTriggerValue * 0.7);
            } else if (leftTriggerValue > 0.0) {
                setShooterPower(-leftTriggerValue * 0.65);
            } else {
                setShooterPower(0);
            }
        }

        // Auto shoot sequence trigger
        if (gamepad2.y && shootState.equals("IDLE")) {
            shootState = "SPINUP";
            shooterTimer.reset();
            shootingActive = true;
            // telemetry.speak("Shooting Sequence Started", "en", "US");
        }

        // State Machine
        switch (shootState) {
            case "SPINUP":
                setShooterPower(-0.7);
                if (shooterTimer.seconds() > 0.5) {
                    shootState = "FEED";
                    shooterTimer.reset();
                }
                break;
            case "FEED":
                conveyorMotor.setPower(1);
                setShooterPower(-0.7);
                if (shooterTimer.seconds() > 2.5) {
                    shootState = "COOLDOWN";
                    shooterTimer.reset();
                }
                break;
            case "COOLDOWN":
                setShooterPower(0);
                conveyorMotor.setPower(0);
                if (shooterTimer.seconds() > 1) {
                    shootState = "IDLE";
                    shootingActive = false;
                    // telemetry.speak("Ready", "en", "US");
                }
                telemetry.addLine(shootState);
                break;
        }
/*
        // Shoot State Change
        if (!shootState.equals(lastShootState)) {
            telemetry.speak(shootState, "en", "US");
            lastShootState = shootState;
        }

 */

        // Telemetry
        telemetry.addData("Shooter State", shootState);
        telemetry.addData("Shooter Power", "%.2f", shooterLeft.getPower());
        telemetry.addData("Conveyor Power", "%.2f", conveyorMotor.getPower());
    }

    /**
     * Handles intake motor.
     * Right bumper runs intake forward,
     * Left bumper runs reverse.
     */
    private void intakeSubsystem() {
        boolean intakeForward;
        boolean intakeReverse;

        intakeForward = gamepad2.right_bumper;
        intakeReverse = gamepad2.left_bumper;
/*
        if (intakeForward && !lastIntakeForward) {
            telemetry.speak("Intake on", "en", "US");
        }

 */

        if (intakeReverse) {
            intakeMotor.setPower(1);
        } else if (intakeForward) {
            intakeMotor.setPower(-1);
        } else {
            intakeMotor.setPower(0);
        }
/*
        lastIntakeForward = intakeForward;
        lastIntakeReverse = intakeReverse;

 */

        telemetry.addData("Intake Power", "%.2f", intakeMotor.getPower());
    }

    /**
     * Controls conveyor (feeder) movement.
     * Gamepad1 A = forward,
     * Gamepad1 B = reverse.
     */
    private void conveyorSubsystem() {
        boolean conveyorForward = gamepad2.a;
        boolean conveyorReverse = gamepad2.b;
/*
        if (conveyorForward && !lastConveyorForward) {
            telemetry.speak("Conveyor forward", "en", "US");
        }

        if (conveyorReverse && !lastConveyorReverse) {
            telemetry.speak("Conveyor reverse", "en", "US");
        }

 */

        if (conveyorReverse) {
            conveyorMotor.setPower(-1);
        } else if (conveyorForward) {
            conveyorMotor.setPower(1);
        } else {
            conveyorMotor.setPower(0);
        }
/*
        lastConveyorForward = conveyorForward;
        lastConveyorReverse = conveyorReverse;

 */

        telemetry.addData("Conveyor Manual", conveyorMotor.getPower() != 0 ? "Active" : "Off");
    }

    private void liftSubsystem () {
        double basePower;
        int leftPosition;
        int rightPosition;
        int positionDifference;
        double correction;
        double leftPower;
        double rightPower;

        basePower = 0.0;

        if (gamepad1.right_bumper) {
            liftMotorLeft.setPower(1.0);
            liftMotorRight.setPower(1.0);
        } else if (gamepad1.left_bumper) {
           liftMotorLeft.setPower(-1.0);
           liftMotorRight.setPower(-1.0);
        } else {
            liftMotorLeft.setPower(0);
            liftMotorRight.setPower(0);
        }

        /*

        leftPosition = liftMotorLeft.getCurrentPosition();
        rightPosition = liftMotorRight.getCurrentPosition();
        positionDifference = leftPosition - rightPosition;

        correction = 0.0;
        if (Math.abs(positionDifference) > levelToleranceTicks && Math.abs(basePower) > 0.01) {
            correction = levelCorrectionGain * positionDifference;
        }

        leftPower = basePower - correction;
        rightPower = basePower + correction;

        liftMotorLeft.setPower(Range.clip(leftPower, -1.0, 1.0));
        liftMotorRight.setPower(Range.clip(leftPower, -1.0, 1.0));
        */


        telemetry.addData("Lift Base Power", "%.2f", basePower);
        /*
        telemetry.addData("Lift Left Pos", leftPosition);
        telemetry.addData("Lift Right Pos", rightPosition);
        telemetry.addData("Lift Diff (L-R)", positionDifference);
        telemetry.addData("Lift Correction", "%.3f", correction);
        telemetry.addData("Lift Left Power", "%.2f", leftPower);
        telemetry.addData("Lift Right Power", "%.2f", rightPower);

         */

    }

    /**
     * Rotates CR servo turntable to a specific 120° sector.
     */
    private void turnToSector(int targetSector, boolean toIntake) {
        double targetPosition;
        double offset;
        double difference;
        double steps;
        int direction;
        long totalTimeMs;

        offset = toIntake ? 0.0 : sectorOffsetForFeed;
        targetPosition = targetSector + offset;

        difference = targetPosition - currentPosition;

        // Choose Shortest Path
        if (difference > sectorsPerCircle / 2.0) {
            difference -= sectorsPerCircle;
        } else if (difference < -sectorsPerCircle / 2.0) {
            difference += sectorsPerCircle;
        }

        steps = Math.abs(difference);
        direction = (difference > 0) ? 1 : ((difference < 0) ? -1 : 0);

        if (direction == 0) {
            return;
        }

        totalTimeMs = Math.round(steps * sectorTime);

        // String targetText = toIntake ? "Intake" : "Feed";
        // telemetry.speak("Turning sector " + targetSector + " to " + targetText, "en", "US");

        sectorTimer.reset();
        turnTableServo.setPower(direction);
        while (sectorTimer.milliseconds() < totalTimeMs && opModeIsActive()) {
            // Wait
        }
        turnTableServo.setPower(0);

        currentPosition += difference;

        currentPosition = (currentPosition % sectorsPerCircle + sectorsPerCircle) % sectorsPerCircle;

        turnTableSector = targetSector;
    }

    /**
     * Controls conveyor (feeder) movement.
     * Gamepad1 A = forward,
     * Gamepad1 B = reverse.
     */
    private void turnTableSubsystem() {
        // Manual Control
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            turnTableServo.setPower(-1.0);
        } else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            turnTableServo.setPower(1.0);
        } else if (!isTurning) {
            turnTableServo.setPower(0);
        }

        // Sector Buttons
        boolean cycleButton = gamepad2.x;
        boolean feedMode = gamepad2.right_bumper;

        if (cycleButton && !lastCycleButton && !isTurning) {
            turnTableSector = (turnTableSector + 1) % 3;
            turnToSector(turnTableSector, !feedMode);
        }
        lastCycleButton = cycleButton;

        // Non-blocking turn update
        if (isTurning && sectorTimer.milliseconds() >= 0) {
            turnTableServo.setPower(0.0);
            isTurning = false;
        }

        // Determine orientation
        double fractional = currentPosition - Math.floor(currentPosition);
        String orientation = (fractional < 0.25 || fractional >= 0.75) ? "Intake" : "Feed";

        telemetry.addData("Turntable Sector", turnTableSector);
        telemetry.addData("Position (sectors)", "%.2f", currentPosition);
        telemetry.addData("Orientation", "%s", orientation);
        telemetry.addData("Target Mode", feedMode ? "FEED" : "INTAKE");
        telemetry.addData("Servo Power", "%.2f", turnTableServo.getPower());

    }
}