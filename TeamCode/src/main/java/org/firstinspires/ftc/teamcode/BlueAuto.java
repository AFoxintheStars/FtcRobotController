package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import mechanisms.Alliance;

@Autonomous(name = "Auto – Blue")
public class BlueAuto extends VORTEX_AutoBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
