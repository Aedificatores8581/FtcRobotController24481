package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import com.qualcomm.hardware.limelightvision.Limelight3A;

/*
 * VISION - the Limelight camera.
 *
 * Put here:
 *  - Limelight setup in init() (start it, pick the pipeline)
 *  - update() that reads the latest result once per loop
 *  - hasTarget(), getTx(), getTy(), setPipeline(int)
 * Do NOT put here: turning the robot. AimController turns tx into a turn power.
 */
public class Vision implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private Limelight3A limelight;

    public Vision(HardwareMap hardwareMap, Telemetry telemetry) {
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

    // TODO: hasTarget(), getTx(), getTy(), setPipeline(int)

    /**
     * Is the hive tipped? Used by the autos to pick a plan.
     * TODO: check the AprilTag the camera sees (which tag id means tipped?).
     */
    public boolean isHiveTipped() {
        return false;
    }
}
