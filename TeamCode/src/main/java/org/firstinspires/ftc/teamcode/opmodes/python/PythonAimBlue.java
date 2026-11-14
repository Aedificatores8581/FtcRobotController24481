package org.firstinspires.ftc.teamcode.opmodes.python;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/** Auto-aim that only targets BLUE AprilTag clusters (Limelight pipeline 3). */
@TeleOp(name = "Python Aim Blue", group = "Python")
public class PythonAimBlue extends PythonAimBase {

    @Override
    protected int getAlliance() {
        return ALLIANCE_BLUE;
    }
}