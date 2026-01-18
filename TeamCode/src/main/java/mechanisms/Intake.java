package mechanisms;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Intake {
    private DcMotorEx intakeMotor;

    // ==== INTAKE POWER SETTINGS =====
    private static final double INTAKE_POWER = 0.85;
    private static final double OUTTAKE_POWER = -0.75;
    private static final double IDLE_POWER = 0.0;

    // ===== CURRENT LIMIT SETTINGS =====
    private static final double WARNING_CURRENT_AMPS = 5.5;
    private static final double MAX_CURRENT_AMPS = 7.0;

    // Power reduction when overloaded
    private static final double REDUCED_POWER_SCALE = 0.4;

    public void init(HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "intake_motor");

        // Set motor direction
        intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);

        // We don't need encoder for intake, just power
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // BRAKE uses more power but holds position better
        // FLOAT lets it coast and uses less power when stopped
        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        // Start with no power
        intakeMotor.setPower(IDLE_POWER);
    }

    /**
     * Controls the intake motor based on gamepad input.
     * Right bumper: intake (pull in)
     * Left bumper: outtake (push out)
     * Both or neither: idle
     */
    public void controlIntake(boolean rightBumper, boolean leftBumper) {
        double desiredPower = IDLE_POWER;

        if (rightBumper) {
            // Intake - pull game piece in
            desiredPower = INTAKE_POWER;
        }
        else if (leftBumper) {
            // Outtake - push game piece out
            desiredPower = OUTTAKE_POWER;
        }

        applyCurrentLimit(desiredPower);
    }

    /**
     * Applies software current limiting to protect battery & hub
     */
    private void applyCurrentLimit(double desiredPower) {

        double current = intakeMotor.getCurrent(CurrentUnit.AMPS);

        if (current >= MAX_CURRENT_AMPS) {
            intakeMotor.setPower(0.0);
        }
        else if (current >= WARNING_CURRENT_AMPS) {
            // Reduce power to prevent brownout
            intakeMotor.setPower(desiredPower * REDUCED_POWER_SCALE);
        }
        else {
            intakeMotor.setPower(desiredPower);
        }
    }

    /**
     * For telemetry
     */
    public double getCurrentDraw() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS);
    }
}