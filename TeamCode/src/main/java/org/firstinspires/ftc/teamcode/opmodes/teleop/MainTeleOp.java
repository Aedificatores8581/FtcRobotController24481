package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;

@TeleOp(name = "Main TeleOp", group = "Main")
public class MainTeleOp extends OpMode {

    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.init();
    }

    @Override
    public void loop() {
        // TODO: read gamepads and call robot.* methods
        robot.update();
    }

    @Override
    public void stop() {
        robot.stop();
    }
}
