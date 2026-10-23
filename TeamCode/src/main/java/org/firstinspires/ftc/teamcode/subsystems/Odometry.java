package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;

/*
 * ODOMETRY - where the robot is on the field (goBILDA Pinpoint).
 *
 * Put here:
 *  - Pinpoint setup in init() (pod offsets and directions from RobotConstants)
 *  - update() that calls pinpoint.update() every loop
 *  - getX(), getY(), getHeading(), resetPose()
 * Do NOT put here: driving. Other code reads the pose and decides what to do.
 */
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
