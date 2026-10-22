package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.util.Alliance;

@Disabled // TODO: remove when ready
@Autonomous(name = "Auto Red Near", group = "Main")
public class AutoRedNear extends AutoBase {

    @Override
    protected Alliance getAlliance() {
        return Alliance.RED;
    }

    @Override
    protected void runRoutine() {
        // TODO
    }
}
