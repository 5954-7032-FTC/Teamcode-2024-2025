package org.firstinspires.ftc.teamcode.subsystems;

public interface MecanumDrive extends SubSystem,DriveSystem {
    void moveRect(double forward, double lateral, double rotate);
    void movePolar(double power, double angle, double rotate);
    void setMotorSpeeds(double [] speeds);

    void setBrakes();
    void clearBrakes();

}
