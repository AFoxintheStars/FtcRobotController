package mechanisms.tests;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestBenchColor {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "color_sensor");
        colorSensor.setGain(15);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;

        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("Red", normRed);
        telemetry.addData("Green", normGreen);
        telemetry.addData("Blue", normBlue);

        /*
        red, green, blue
        RED = >0.65, <0.35, <0.3,
        YELLOW = >1, >1, <0.7
        BLUE = <0.15, <0.4, >0.5
         */

        if (normRed > 0.65 && normGreen < 0.35 && normBlue < 0.3) {
            return DetectedColor.RED;
        }
        else if (normRed > 1 && normGreen > 1 && normBlue < 0.7) {
            return DetectedColor.YELLOW;
        }
        else  if (normRed < 0.15 && normGreen < 0.4 && normBlue > 0.5) {
            return DetectedColor.BLUE;
        }
        else {
            return DetectedColor.UNKNOWN;
        }
    }
}
