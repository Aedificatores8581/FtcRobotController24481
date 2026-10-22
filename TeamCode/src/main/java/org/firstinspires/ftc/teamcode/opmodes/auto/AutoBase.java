package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;

/* Shared setup and helpers for all autonomous OpModes. */
public abstract class AutoBase extends LinearOpMode {

    protected Robot robot;

    protected abstract Alliance getAlliance();

    /** Each auto implements its routine here. */
    protected abstract void runRoutine();

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();

        waitForStart();
        if (opModeIsActive()) runRoutine();

        robot.stop();
    }

    // TODO: driveTo(x, y, heading), waitFor(seconds) that keeps calling robot.update()
}
