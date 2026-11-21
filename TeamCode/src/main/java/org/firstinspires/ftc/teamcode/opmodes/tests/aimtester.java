package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoBase;

@Autonomous(name = "aimtester", group = "aim")
public class aimtester extends AutoBase {

    @Override
    protected void runRoutine() {
        // start the flywheel, then scan for the hive tags from the starting position
        robot.launcher.spinUp();


        // Wait in a loop until launcher is ready (NOT ready -> keep updating)
        while (opModeIsActive() && !robot.launcher.isReady()) {
            robot.update();
        }
        robot.launcher.feed();
        waitSeconds(1);
        robot.intake.on();
        waitSeconds(3);
        robot.update();

        robot.launcher.stopFlywheel();
        robot.intake.off();
    }
}
