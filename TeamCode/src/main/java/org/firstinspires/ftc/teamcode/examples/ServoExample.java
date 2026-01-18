package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import mechanisms.tests.TestBenchServo;

@TeleOp
@Disabled
public class ServoExample extends OpMode {

    TestBenchServo bench = new TestBenchServo();
    double leftTrigger, rightTrigger;

    @Override
    public void init() {
        bench.init(hardwareMap);
        rightTrigger = 0.0;
        leftTrigger = 0.0;
    }

    @Override
    public void loop() {
        leftTrigger = gamepad1.left_trigger;
        rightTrigger = gamepad1.right_trigger;

        bench.setServoRot(leftTrigger);
        bench.setServoPos(rightTrigger);

        if (gamepad1.a) {
            bench.setServoPos(0);
        }
        else {
            bench.setServoPos(1.0);
        }

        if (gamepad1.b) {
            bench.setServoRot(1.0);
        }
        else {
            bench.setServoRot(0);
        }

    }
}
