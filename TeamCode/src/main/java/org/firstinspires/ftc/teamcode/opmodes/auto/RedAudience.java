package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

// Red audience side, tags 34-37. Mirror of BlueAudience. Not tested.
@Autonomous(name = "RedAudience", group = "Main")
public class RedAudience extends AutoBase {

    @Override
    protected void runRoutine() {
        shootAtHive(RobotConstants.TAG_RED_AUDIENCE_MIN, RobotConstants.TAG_RED_AUDIENCE_MAX);
        leaveAndPark();
    }

    private void leaveAndPark() {
        double p = RobotConstants.AUTO_POWER;

        driveInches(24, p);
        turnToDegrees(90, p);
        driveInches(41, p);
    }
}
