package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import com.qualcomm.robotcore.hardware.DcMotor;

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
