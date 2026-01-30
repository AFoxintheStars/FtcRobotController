package mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climber {

    private DcMotor leftClimber;
    private DcMotor rightClimber;

    private static final double DEADZONE = 0.05;

    // Direction state
    private boolean descending = false;
    private boolean lastBackState = false;

    public void init(HardwareMap hwMap) {
        leftClimber = hwMap.get(DcMotor.class, "climber_left");
        rightClimber = hwMap.get(DcMotor.class, "climber_right");

        leftClimber.setDirection(DcMotorSimple.Direction.FORWARD);
        rightClimber.setDirection(DcMotorSimple.Direction.FORWARD);

        leftClimber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightClimber.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftClimber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightClimber.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        stop();
    }

    /**
     * Independent climber control
     */
    public void controlClimber(double leftTrigger,
                               double rightTrigger,
                               boolean backButton) {

        if (backButton && !lastBackState) {
            descending = !descending;
        }
        lastBackState = backButton;

        double directionMultiplier = descending ? -1.0 : 1.0;

        double leftPower = (leftTrigger > DEADZONE)
                ? leftTrigger * directionMultiplier
                : 0.0;

        double rightPower = (rightTrigger > DEADZONE)
                ? rightTrigger * directionMultiplier
                : 0.0;

        leftClimber.setPower(leftPower);
        rightClimber.setPower(rightPower);
    }

    public void stop() {
        leftClimber.setPower(0.0);
        rightClimber.setPower(0.0);
    }

    public void addTelemetry(Telemetry telemetry) {
        telemetry.addData("Climber Mode", descending ? "DESCEND" : "CLIMB");
        telemetry.addData("Left Power", "%.2f", leftClimber.getPower());
        telemetry.addData("Right Power", "%.2f", rightClimber.getPower());
    }
}
