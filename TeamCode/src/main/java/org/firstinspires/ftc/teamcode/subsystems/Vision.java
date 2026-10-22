package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import com.qualcomm.hardware.limelightvision.Limelight3A;

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
}
