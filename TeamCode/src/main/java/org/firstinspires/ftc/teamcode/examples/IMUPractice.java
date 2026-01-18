package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import mechanisms.tests.TestBenchIMU;

@TeleOp
@Disabled
public class IMUPractice extends OpMode {
    TestBenchIMU testBenchIMU = new TestBenchIMU();
    double heading;

    @Override
    public void init() {
        testBenchIMU.init(hardwareMap);
    }

    @Override
    public void loop() {
        heading = testBenchIMU.getHeading(AngleUnit.DEGREES);
        telemetry.addData("Heading", testBenchIMU.getHeading(AngleUnit.DEGREES));

        if (heading < 0.5 && heading > -0.5) {
            testBenchIMU.setMotor(0.0);
        }
        else if (heading > 0.5) {
            testBenchIMU.setMotor(0.5);
        }
        else {
            testBenchIMU.setMotor(-0.5);
        }
    }
}
