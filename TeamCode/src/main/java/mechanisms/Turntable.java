package mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Turntable {

    private CRServo servo;

    // ===== TUNING =====
    private static final double TURN_POWER = 0.3;

    private double currentPower = 0.0;

    // ===============================
    // INIT
    // ===============================
    public void init(HardwareMap hwMap) {
        servo = hwMap.get(CRServo.class, "turntable_servo");
        stop();
    }

    // ===============================
    // MANUAL CONTROL
    // ===============================
    public void handleManualControl(boolean leftBumper, boolean rightBumper) {

        if (leftBumper && !rightBumper) {
            setPower(+TURN_POWER);
        }
        else if (rightBumper && !leftBumper) {
            setPower(-TURN_POWER);
        }
        else {
            stop();
        }
    }

    // ===============================
    // CORE CONTROL
    // ===============================
    public void setPower(double power) {
        currentPower = power;
        servo.setPower(power);
    }

    public void stop() {
        currentPower = 0.0;
        servo.setPower(0.0);
    }

    // ===============================
    // STATE QUERY
    // ===============================
    public boolean isMoving() {
        return Math.abs(currentPower) > 0.01;
    }

    public double getPower() {
        return currentPower;
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Turntable Power", "%.2f", servo.getPower());
    }
}
