package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import mechanisms.tests.RobotLocation;

@TeleOp
@Disabled
public class UseRobotLocation extends OpMode {
    RobotLocation robotLocation = new RobotLocation(0);

    @Override
    public void init() {
        robotLocation.setAngle(0);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            robotLocation.turnRobot(0.1);
        }
        else if (gamepad1.b) {
            robotLocation.turnRobot(-0.1);
        }

        if (gamepad1.dpad_left) {
            robotLocation.changeX(0.1);
        }
        else if (gamepad1.dpad_right) {
            robotLocation.changeX(-0.1);
        }

        if (gamepad1.dpad_up) {
            robotLocation.changeY(0.1);
        }
        else if (gamepad1.dpad_down) {
            robotLocation.changeY(-0.1);
        }

        telemetry.addData("Heading", robotLocation.getHeading());
        telemetry.addData("Angle", robotLocation.getAngle());
        telemetry.addData("X value", robotLocation.getX());
        telemetry.addData("Y value", robotLocation.getY());
    }
}
