package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.RobotConstants;

// Builds the Pedro follower. It drives the motors and reads the Pinpoint, so don't use it with Robot.drive or Robot.o
// creates a Constansts object for the Pedro follower
public final class Constants {

    // Prevent instantiation of this utility class
    private Constants() {}

    // Creates and returns a configured Pedro follower instance
    // constants come from RobotConstants and are used to configure the follower
    public static Follower create(HardwareMap hardwareMap) {
        MecanumConfig mecanum = new MecanumConfig(c -> {
            c.frontLeftName.set(RobotConstants.FRONT_LEFT_MOTOR);
            c.backLeftName.set(RobotConstants.BACK_LEFT_MOTOR);
            c.frontRightName.set(RobotConstants.FRONT_RIGHT_MOTOR);
            c.backRightName.set(RobotConstants.BACK_RIGHT_MOTOR);
            c.frontLeftDirection.set(RobotConstants.FRONT_LEFT_DIRECTION);
            c.backLeftDirection.set(RobotConstants.BACK_LEFT_DIRECTION);
            c.frontRightDirection.set(RobotConstants.FRONT_RIGHT_DIRECTION);
            c.backRightDirection.set(RobotConstants.BACK_RIGHT_DIRECTION);
        });

        PinpointConfig pinpoint = new PinpointConfig(c -> {
            c.name.set(RobotConstants.PINPOINT);
            c.podType.set(RobotConstants.POD_TYPE);
            c.xPodDirection.set(RobotConstants.X_POD_DIRECTION);
            c.yPodDirection.set(RobotConstants.Y_POD_DIRECTION);
            c.xPodOffset.set(RobotConstants.PINPOINT_X_OFFSET_MM);
            c.yPodOffset.set(RobotConstants.PINPOINT_Y_OFFSET_MM);
            c.offsetUnits.set(DistanceUnit.MM);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.resetMode.set(PinpointLocalizer.ResetMode.RESET_AND_RECALIBRATE_IMU); // keep still in INIT
        });

        // TODO: foresight is still defaults, paste in the AutoTune values
        return new Follower(
                new PinpointLocalizer(hardwareMap, pinpoint),
                new Mecanum(hardwareMap, mecanum),
                // foresight is used for predictive control of the robot's motion
                new Foresight(new ForesightConfig(c -> { })));
    }
}
