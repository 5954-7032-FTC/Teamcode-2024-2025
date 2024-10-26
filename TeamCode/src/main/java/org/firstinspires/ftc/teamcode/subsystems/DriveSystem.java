package org.firstinspires.ftc.teamcode.subsystems;

public interface DriveSystem {
    void moveRect(double forward, double lateral, double rotate);
    void movePolar(double power, double angle, double rotate);
    void outputTelemetry(TelemetryTypes type);
}

