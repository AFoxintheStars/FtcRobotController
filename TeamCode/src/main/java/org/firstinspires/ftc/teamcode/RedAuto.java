package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import mechanisms.Alliance;

@Autonomous(name = "Auto – Red")
public class RedAuto extends VORTEX_AutoBase {
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}
