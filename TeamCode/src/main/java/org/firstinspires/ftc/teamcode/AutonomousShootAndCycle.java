package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Shoot and Cycle", group="Autonomous")
@Disabled
public class AutonomousShootAndCycle extends LinearOpMode {

    // Hardware
    private DcMotor shooterLeft;
    private DcMotor shooterRight;
    private DcMotor conveyorMotor;
    private DcMotor driveMotorLeft;
    private DcMotor driveMotorRight;
    private CRServo turnTableServo;

    // Timers
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime actionTimer= new ElapsedTime();

    // Constants
    private final double shooterPower = 0.65;
    private final double conveyorFeedPower = 1.0;
    private final double spinupTime = 0.01;
    private final double feedTime = 2;
    private final double coolDownTime = 0.01;
    private final int sectorTimeMS = 560;
    private final double driveForwardPower = 0.6;

    private final double driveTimeSeconds = 1.0;

    @Override
    public void runOpMode() {
        InitializeHardware();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

       for (int i = 1; i <= 4; i++) {
           telemetry.addData("AutoCycle", "Shooting ring %d of 3", i);
           telemetry.update();

           if (i < 4) {
               turnTableToNextSector();
           }

           setShooterPower(shooterPower);
           sleep((long)(spinupTime * 1000));

           conveyorMotor.setPower(conveyorFeedPower);
           sleep((long)(feedTime * 1000));
           conveyorMotor.setPower(0);

           sleep((long)(coolDownTime * 1000));
        }

       setShooterPower(0);

       telemetry.addData("Auto", "Driving Forward");
       telemetry.update();

       driveMotorLeft.setPower(driveForwardPower);
       driveMotorRight.setPower(driveForwardPower);
       sleep((long)(driveTimeSeconds * 1000));
       driveMotorLeft.setPower(0);
       driveMotorRight.setPower(0);

       telemetry.addData("AutoComplete", "Done in %.1f seconds", runtime.seconds());
    }

    private void InitializeHardware() {
        driveMotorRight = hardwareMap.get(DcMotor.class, "driveMotorRight");
        driveMotorLeft = hardwareMap.get(DcMotor.class, "driveMotorLeft");
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        shooterLeft = hardwareMap.get(DcMotor.class, "shooterLeft");
        shooterRight = hardwareMap.get(DcMotor.class, "shooterRight");
        turnTableServo = hardwareMap.get(CRServo.class, "turnTableServo");

        driveMotorRight.setDirection(DcMotor.Direction.REVERSE);
        driveMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        shooterLeft.setDirection(DcMotor.Direction.REVERSE);
        shooterRight.setDirection(DcMotor.Direction.FORWARD);
        conveyorMotor.setDirection(DcMotor.Direction.FORWARD);
        turnTableServo.setDirection(CRServo.Direction.FORWARD);

        driveMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        driveMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void setShooterPower(double power) {
        shooterLeft.setPower(-power);
        shooterRight.setPower(-power);
    }

    private void turnTableToNextSector() {
        actionTimer.reset();
        turnTableServo.setPower(1.0);

        while (actionTimer.milliseconds() < sectorTimeMS && opModeIsActive()) {
            telemetry.addData("Turning", "Next Sector...");
            telemetry.update();
        }

        turnTableServo.setPower(0);
    }
}
