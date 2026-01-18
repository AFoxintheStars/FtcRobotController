package mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Conveyor {

    private DcMotorEx conveyorMotor;

    // ===== MECHANICAL CONSTANTS =====
    private static final double GEAR_RATIO = 20.0; // 20:1 reduction
    private static final double TICKS_PER_REV = 560;
    // Change if using a different motor

    // ===== CONVEYOR SPEED SETTINGS =====
    private static final double CONVEYOR_RPM = 90.0; // output shaft RPM
    private static final double MOTOR_RPM = CONVEYOR_RPM * GEAR_RATIO;

    private static final double TARGET_TICKS_PER_SEC =
            (MOTOR_RPM * TICKS_PER_REV) / 60.0;

    public void init(HardwareMap hwMap) {

        conveyorMotor = hwMap.get(DcMotorEx.class, "conveyor_motor");

        conveyorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        conveyorMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        conveyorMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        stop();
    }

    // ===== CONTROL METHODS =====

    public void forward() {
        conveyorMotor.setVelocity(TARGET_TICKS_PER_SEC);
    }

    public void reverse() {
        conveyorMotor.setVelocity(-TARGET_TICKS_PER_SEC);
    }

    public void stop() {
        conveyorMotor.setVelocity(0);
    }

    // ===== TELEMETRY =====

    public double getVelocity() {
        return conveyorMotor.getVelocity();
    }

    public double getTargetVelocity() {
        return TARGET_TICKS_PER_SEC;
    }
}
