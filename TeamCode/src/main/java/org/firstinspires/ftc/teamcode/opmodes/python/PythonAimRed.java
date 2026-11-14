package org.firstinspires.ftc.teamcode.opmodes.python;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Auto-aim that only targets RED AprilTag clusters (Limelight pipeline 3). */
@TeleOp(name = "Python Aim Red", group = "Python")
public class PythonAimRed extends PythonAimBase {

    @Override
    protected int getAlliance() {
        return ALLIANCE_RED;
    }
}