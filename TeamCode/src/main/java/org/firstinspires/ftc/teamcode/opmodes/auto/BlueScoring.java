package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

@Autonomous(name = "BlueScoring", group = "Main")
public class BlueScoring extends AutoBase {

    // TODO measure on the field: angle that faces the balls, and which side the wall is on
    private static final double GATHER_HEADING_DEGREES = 90; // positive is left
    private static final double WALL_POWER = -RobotConstants.AUTO_POWER; // negative = wall on left

    @Override
    protected void runRoutine() {
        double p = RobotConstants.AUTO_POWER;

        // start the flywheel, then look for the hive tags
        robot.launcher.spinUp();
        log("Scanning for hive tags");

        // tags in view: shoot to tip the hive. Not in view: nothing to shoot at
        if (hiveStillUp()) {
            log("Tags seen, shooting 3");
            shoot(3);
        } else {
            log("No tags seen, waiting 15s");
            waitSeconds(10);
        }

        // the hive takes about 2 seconds to tip
        log("Waiting 2s for hive to tip");
        waitSeconds(2);

        // tags still in view: it did not tip, so gather more balls and shoot again
        // tags gone: it tipped, so just leave and park
        boolean stillUp = robot.vision.isHiveTipped(RobotConstants.TAG_BLUE_SCORING_MIN,
                RobotConstants.TAG_BLUE_SCORING_MAX);
        log("Hive still up after 2s: " + stillUp);

        if (stillUp) {
            // save the shooting spot, gather along the wall, then check and correct back to it
            markPose();
            log("Turning to gather heading " + GATHER_HEADING_DEGREES);
            turnToDegrees(GATHER_HEADING_DEGREES, p);
            log("Gathering along wall");
            gatherAlongWall(WALL_POWER);
            turnToDegrees(0, p);
            driveInches(4, -p);
            checkAndCorrect();

            // look for the tags one more time, only shoot if they are there
            log("Scanning for hive tags before second shot");
            if (hiveStillUp()) {
                log("Tags seen, shooting 3 more");
                shoot(3);
            } else {
                log("No tags seen, skipping second shot");
            }
        }

        log("Leave and park");
        leaveAndPark();
        log("Done");
    }

    // Scan for the hive tags. True means a tag is still in view, so the hive is still up.
    // (Vision calls this "tipped", but a visible tag means it has NOT tipped over.)
    private boolean hiveStillUp() {
        return scanForTip(RobotConstants.TAG_BLUE_SCORING_MIN, RobotConstants.TAG_BLUE_SCORING_MAX);
    }

    // leave and park, then square up on the wall
    private void leaveAndPark() {
        double p = RobotConstants.AUTO_POWER;

        driveInches(12, p);
        turnToDegrees(85, p);
        driveInches(30, p);
        turnToDegrees(0, p);
        driveInches(80, p);
        turnToDegrees(80, p);
        driveInches(10, p);
        driveSeconds(1.5, p);
        driveSeconds(0.2, -p);
    }
}
