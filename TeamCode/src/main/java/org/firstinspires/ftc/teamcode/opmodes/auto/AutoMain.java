package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robot;

@Disabled // TODO: remove when ready
@Autonomous(name = "Auto Main", group = "Main")
public class AutoMain extends LinearOpMode {

    private Robot robot;

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();

        waitForStart();

        // TODO: the autonomous routine goes here
        // Call robot.update() each loop so the subsystems keep running.

        robot.stop();
    }
}
