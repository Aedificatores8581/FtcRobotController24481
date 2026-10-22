package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

public class Odometry implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    // TODO: GoBildaPinpointDriver pinpoint;

    public Odometry(HardwareMap hardwareMap, Telemetry telemetry) {
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

    // TODO: getX(), getY(), getHeading(), resetPose()
}
