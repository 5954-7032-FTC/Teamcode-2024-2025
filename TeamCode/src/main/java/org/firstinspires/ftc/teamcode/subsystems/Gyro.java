package org.firstinspires.ftc.teamcode.subsystems;

public interface Gyro extends SubSystem {
    void resetAngle();

    double getAngle();

    double getAbsoluteAngle();

    double getSteeringCorrection(double desiredHeading, double proportionalGain);

    void holdHeading(double maxTurnSpeed, double heading, double holdTime);
}
