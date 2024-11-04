package org.firstinspires.ftc.teamcode.subsystems;

public interface DriveRobot extends SubSystem {
    void turn(double degrees);

    void turnTo(double degrees);

    void driveReverse(double distance);

    void driveRight(double distance);

    void driveForward(double distance);

    void driveLeft(double distance);

    void stopRobot();
}
