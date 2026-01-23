package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.VORTEXV2_Base;

import mechanisms.Alliance;

@TeleOp(name = "VORTEX - BLUE")
public class VORTEXV2_Blue extends VORTEXV2_Base {
    @Override
    protected Alliance getAlliance() {
        return Alliance.BLUE;
    }
}
