package org.firstinspires.ftc.teamcode.control;

/*
 * PID CONTROLLER - reusable feedback math, no hardware.
 *
 * Put here: calculate(target, current) using kP, kI, kD, and reset().
 * Use it for things the motor controller can't do itself (e.g. heading hold).
 */
public class PIDController {

    public double kP, kI, kD;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public double calculate(double target, double current) {
        // TODO
        return 0;
    }

    public void reset() {
        // TODO
    }
}
