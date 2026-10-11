package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/*
 * Minimal Pinpoint smoke test, adapted from the SDK sample
 * FtcRobotController/.../external/samples/SensorGoBildaPinpoint.java.
 *
 * Uses the driver that ships with the FTC SDK (com.qualcomm.hardware.gobilda).
 * Assumes the Pinpoint is configured on the Driver Hub with the name "pinpoint".
 *
 * Run it, then push the robot around by hand:
 *   - X should increase when the robot moves forward
 *   - Y should increase when the robot moves left
 *   - Heading should increase when the robot turns counter-clockwise
 * If any of those are backwards, flip X_POD_DIRECTION / Y_POD_DIRECTION in RobotConstants.java.
 */
@TeleOp(name = "Pinpoint Test", group = "Sensor")
public class PinpointTest extends OpMode {
    GoBildaPinpointDriver pinpoint;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, RobotConstants.PINPOINT);

        // All Pinpoint settings live in RobotConstants.java so they match what the autonomous uses.
        pinpoint.setOffsets(RobotConstants.X_POD_OFFSET_MM, RobotConstants.Y_POD_OFFSET_MM, DistanceUnit.MM);
        pinpoint.setEncoderResolution(RobotConstants.POD_TYPE);
        pinpoint.setEncoderDirections(RobotConstants.X_POD_DIRECTION, RobotConstants.Y_POD_DIRECTION);

        // Robot must be stationary for this. Zeros position and recalibrates the IMU.
        pinpoint.resetPosAndIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Device version", pinpoint.getDeviceVersion());
    }

    @Override
    public void loop() {
        // One I2C bulk read per loop. Every getter below returns the values cached by this call.
        pinpoint.update();

        if (gamepad1.a) {
            pinpoint.resetPosAndIMU();
        }

        Pose2D pose = pinpoint.getPosition();

        telemetry.addLine("Push the robot around. Press A to zero.");
        telemetry.addData("X (in)", "%.2f", pose.getX(DistanceUnit.INCH));
        telemetry.addData("Y (in)", "%.2f", pose.getY(DistanceUnit.INCH));
        telemetry.addData("Heading (deg)", "%.1f", pose.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Pinpoint status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint Hz", "%.0f", pinpoint.getFrequency());
    }
}
