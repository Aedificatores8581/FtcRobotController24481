package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

// Red scoring side, tags 30-33. Mirror of BlueScoring. Not tested.
@Autonomous(name = "RedScoring", group = "Main")
public class RedScoring extends AutoBase {

    @Override
    protected void runRoutine() {
        shootAtHive(RobotConstants.TAG_RED_SCORING_MIN, RobotConstants.TAG_RED_SCORING_MAX);
        leaveAndPark();
    }

    private void leaveAndPark() {
        double p = RobotConstants.AUTO_POWER;

        driveInches(12, p);
        turnToDegrees(-90, p);
        driveInches(30, p);
        turnToDegrees(0, p);
        driveInches(79, p);
        turnToDegrees(-80, p);
        driveInches(10, p);
        driveSeconds(1.5, p);    // push into the wall to square up
        driveSeconds(0.3, -p);   // back out
    }
}
