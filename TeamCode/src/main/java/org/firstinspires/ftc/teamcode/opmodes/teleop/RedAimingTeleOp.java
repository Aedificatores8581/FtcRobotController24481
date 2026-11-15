package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

/** Main TeleOp for the RED alliance: drives manually and auto-aims at RED tags. See AimingTeleOpBase. */
@TeleOp(name = "!!Red Aiming TeleOp", group = "Competition")
public class RedAimingTeleOp extends AimingTeleOpBase {

    @Override
    protected Alliance alliance() {
        return Alliance.RED;
    }
}
