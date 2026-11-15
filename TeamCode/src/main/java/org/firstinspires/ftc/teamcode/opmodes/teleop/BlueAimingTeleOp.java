package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

/** Main TeleOp for the BLUE alliance: drives manually and auto-aims at BLUE tags. See AimingTeleOpBase. */
@TeleOp(name = "!!Blue Aiming TeleOp", group = "Competition")
public class BlueAimingTeleOp extends AimingTeleOpBase {

    @Override
    protected Alliance alliance() {
        return Alliance.BLUE;
    }
}
