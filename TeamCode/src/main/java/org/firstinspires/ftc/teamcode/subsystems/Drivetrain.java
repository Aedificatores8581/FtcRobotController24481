package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * DRIVETRAIN - the four mecanum wheel motors.
 *
 * Put here:
 *  - motor setup in init() (names from RobotConstants, directions, brake mode)
 *  - drive(forward, strafe, turn) with the mecanum math
 *  - optional: field-centric drive (needs heading from Odometry)
 * Do NOT put here: gamepad reading (that belongs in the TeleOp).
 */
public class Drivetrain implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private DcMotor frontLeft, backLeft, frontRight, backRight;

    public Drivetrain(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    @Override
    public void init() {
        // TODO: get hardware from hardwareMap using RobotConstants names
    }

    @Override
    public void update() {
        // TODO
    }

    @Override
    public void stop() {
        // TODO: set outputs to zero
    }

    /** Robot-centric mecanum drive. */
    public void drive(double forward, double strafe, double turn) {
        // TODO: mecanum math
    }
}
