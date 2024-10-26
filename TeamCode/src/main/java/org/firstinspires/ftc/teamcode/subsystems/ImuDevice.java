package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ImuOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.SubSystem;

public class ImuDevice implements SubSystem {
    protected final IMU imu;

    protected double headingOffset;

    public ImuDevice(IMU imu) {
        this.imu = imu;
        IMU.Parameters myIMUparameters;

        myIMUparameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );

        imu.initialize(myIMUparameters);

    }

    public double getRawHeading() {
        //return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        return imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.YZX,AngleUnit.DEGREES).firstAngle;
        //return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
    }
    public Orientation getOrientation() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.YZX, AngleUnit.DEGREES);
        //return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    }

    public double getHeading() {
        return getRawHeading() - headingOffset;
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        headingOffset = getRawHeading();
     }


}
