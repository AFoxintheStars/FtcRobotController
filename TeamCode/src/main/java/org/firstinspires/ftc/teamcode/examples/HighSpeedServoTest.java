package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp
@Disabled
public class HighSpeedServoTest extends OpMode {

    private CRServo crServo;

    @Override
    public void init() {
        crServo = hardwareMap.get(CRServo.class, "turnTableServo");

    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            crServo.setPower(1.0);
        }
        else if (gamepad1.b) {
            crServo.setPower(-1.0);
        }
        else {
            crServo.setPower(0.0);
        }
    }
}
