package org.firstinspires.ftc.teamcode.util;

public interface PID {
    double update(double current);

    public void reset(double target);
    public double getPTerm(double current);

    public double getITerm(double current);

    public double getDTerm(double current);
}
