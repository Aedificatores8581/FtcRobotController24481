package org.firstinspires.ftc.teamcode.control;

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
