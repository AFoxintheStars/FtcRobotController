package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import mechanisms.tests.TestBenchColor;

@TeleOp
@Disabled
public class ColorSensorTest extends OpMode {
    TestBenchColor bench = new TestBenchColor();
    TestBenchColor.DetectedColor detectedColor;

    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop() {
        bench.getDetectedColor(telemetry);
        telemetry.addData("Color Detected", detectedColor);
    }
}
