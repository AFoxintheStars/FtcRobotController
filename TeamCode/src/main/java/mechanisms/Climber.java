package mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climber {

    private DcMotor leftClimber;
    private DcMotor rightClimber;

    // Adjustable constants
    private static final double CLIMB_POWER = 1.0;
    private static final double DESCEND_POWER = -0.8;
    private static final double DEADZONE = 0.1;

    private boolean climbingAllowed = false;

    public void init(HardwareMap hwMap) {
        leftClimber = hwMap.get(DcMotor.class, "climber_left");
        rightClimber = hwMap.get(DcMotor.class, "climber_right");

        leftClimber.setDirection(DcMotorSimple.Direction.FORWARD);
        rightClimber.setDirection(DcMotorSimple.Direction.REVERSE);

        leftClimber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftClimber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightClimber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }

    /**
     * Main control method - call every loop
     * @param leftTrigger  left trigger value [0..1]
     * @param rightTrigger right trigger value [0..1]
     */
    public void controlClimber(double leftTrigger, double rightTrigger) {
        if (!climbingAllowed) {
            stop();
            return;
        }

        double leftPower = 0.0;
        double rightPower = 0.0;

        if (leftTrigger > DEADZONE) {
            leftPower = CLIMB_POWER;
            rightPower = CLIMB_POWER;
        }
        else if (rightTrigger > DEADZONE) {
            leftPower = DESCEND_POWER;
            rightPower = DESCEND_POWER;
        }

        leftClimber.setPower(leftPower);
        rightClimber.setPower(rightPower);
    }

    /**
     * Emergency stop / default state
     */
    public void stop() {
        leftClimber.setPower(0.0);
        rightClimber.setPower(0.0);
    }

    /**
     * Call this at the start of endgame / when climbing is allowed
     * (e.g., when timer reaches 90 seconds remaining)
     */
    public void enableClimbing() {
        climbingAllowed = true;
    }

    /**
     * Call this to disable climbing (safety during normal match)
     */
    public void disableClimbing() {
        climbingAllowed = false;
        stop();
    }

    /**
     * Optional: for telemetry/debug
     */
    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Climber Allowed", climbingAllowed ? "YES" : "NO (match mode)");
        telemetry.addData("Left Climber Power", "%.2f", leftClimber.getPower());
        telemetry.addData("Right Climber Power", "%.2f", rightClimber.getPower());
    }
}