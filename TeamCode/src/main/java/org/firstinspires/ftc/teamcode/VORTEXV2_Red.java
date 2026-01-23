package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VORTEXV2_Base;

import mechanisms.Alliance;

@TeleOp(name = "VORTEX - RED")
public class VORTEXV2_Red extends VORTEXV2_Base {
    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }
}
