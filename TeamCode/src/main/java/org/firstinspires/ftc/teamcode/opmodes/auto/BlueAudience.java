package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotConstants;

// Blue audience side, tags 38-41. Not tested.
@Autonomous(name = "BlueAudience", group = "Main")
public class BlueAudience extends AutoBase {

    @Override
    protected void runRoutine() {
        shootAtHive(RobotConstants.TAG_BLUE_AUDIENCE_MIN, RobotConstants.TAG_BLUE_AUDIENCE_MAX);
        leaveAndPark();
    }

    private void leaveAndPark() {
        double p = RobotConstants.AUTO_POWER;

        driveInches(24, p);
        turnToDegrees(-90, p);
        driveInches(41, p);
    }
}
