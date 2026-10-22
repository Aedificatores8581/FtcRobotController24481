package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotConstants;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Launcher implements Subsystem {

    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;

    private DcMotorEx flywheel;
    private Servo feeder;

    public enum State { IDLE, SPINNING_UP, READY, FEEDING }
    private State state = State.IDLE;

    public Launcher(HardwareMap hardwareMap, Telemetry telemetry) {
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

    // TODO: spinUp(double rpm), feed(), cancel(), isReady(), getRpm()
}
