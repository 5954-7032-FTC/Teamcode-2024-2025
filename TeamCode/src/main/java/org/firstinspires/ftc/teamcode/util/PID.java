package org.firstinspires.ftc.teamcode.util;

public interface PID {
    double update(double current);

    void reset(double target);
    double getPTerm(double current);

    double getITerm(double current);

    double getDTerm(double current);
}
