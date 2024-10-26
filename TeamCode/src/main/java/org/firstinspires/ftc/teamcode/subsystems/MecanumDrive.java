package org.firstinspires.ftc.teamcode.subsystems;

public interface MecanumDrive extends SubSystem,DriveSystem {
    public void moveRect(double forward, double lateral, double rotate);
    public void movePolar(double power, double angle, double rotate);
    public void setMotorSpeeds(double [] speeds);

    public void setBrakes();
    public void clearBrakes();

    public double getSpeedForward();
}
