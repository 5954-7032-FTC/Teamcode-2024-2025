package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class MecanumDriveGyroImpl extends MecanumDriveSubsystemImpl {
    protected IMU _imu;

    protected Telemetry.Item T_IMU;
    public MecanumDriveGyroImpl(MecanumDriveParameters params) {
        super(params);
        _imu = params.imu;
        T_IMU = _telemetry.addData("IMU", "(0,0,0)");
    }

    @Override
    public void outputTelemetry(TelemetryTypes type) {
        switch (type) {
            case IMU:
                Orientation orientation = _imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                T_IMU.setValue("("+orientation.firstAngle+","+orientation.secondAngle+","+orientation.thirdAngle+")");
                break;
            default:
                super.outputTelemetry(type);
        }
    }

    @Override
    public void movePolar(double power, double angle, double rotate) {
        super.movePolar(power, angle, rotate);
        outputTelemetry(TelemetryTypes.IMU);
    }
}
