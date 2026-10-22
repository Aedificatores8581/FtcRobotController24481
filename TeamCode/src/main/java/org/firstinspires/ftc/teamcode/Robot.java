package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Odometry;
import org.firstinspires.ftc.teamcode.subsystems.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

/*
 * Owns every subsystem. OpModes create one Robot and call init(), update(), stop().
 */
public class Robot {

    public final Drivetrain drive;
    public final Odometry odo;
    public final Launcher launcher;
    public final Intake intake;
    public final Vision vision;

    private final Subsystem[] subsystems;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        drive    = new Drivetrain(hardwareMap, telemetry);
        odo      = new Odometry(hardwareMap, telemetry);
        launcher = new Launcher(hardwareMap, telemetry);
        intake   = new Intake(hardwareMap, telemetry);
        vision   = new Vision(hardwareMap, telemetry);

        subsystems = new Subsystem[] {drive, odo, launcher, intake, vision};
    }

    public void init() {
        for (Subsystem s : subsystems) s.init();
    }

    /** Call once per loop. */
    public void update() {
        for (Subsystem s : subsystems) s.update();
    }

    public void stop() {
        for (Subsystem s : subsystems) s.stop();
    }
}
