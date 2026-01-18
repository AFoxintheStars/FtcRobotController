package mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {
    private IMU imu;
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    public void init(HardwareMap hwMap) {
        imu = hwMap.get(IMU.class, "imu");
        leftDrive = hwMap.get(DcMotor.class, "drive_left");
        rightDrive = hwMap.get(DcMotor.class, "drive_right");

        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double getHeading(AngleUnit angleUnit) {
        return imu.getRobotYawPitchRollAngles().getYaw(angleUnit);
    }

    public void teleopDrive(double forward, double turn) {
        double leftPower = forward - turn;
        double rightPower = forward + turn;

        double max = Math.max(Math.abs(leftPower), Math.abs(rightPower));
        if (max > 1.0) {
            leftPower /= max;
            rightPower /= max;
        }

        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);
    }
}