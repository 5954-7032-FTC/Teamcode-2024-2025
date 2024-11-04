package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.subsystems.ImuDevice;
import org.firstinspires.ftc.teamcode.util.RobotDevices;


//@TeleOp(name = "TestImu")
public class TestImu extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ImuDevice imu = new ImuDevice(RobotDevices.getDevices(hardwareMap).imu);
        waitForStart();

        Telemetry.Item [] angles_telemetry = {
            telemetry.addData("first", 0.0),
                    telemetry.addData("second", 0.0),
                    telemetry.addData("third", 0.0)
        };

        while (opModeIsActive()) {
            Orientation angles = imu.getOrientation();
            angles_telemetry[0].setValue(angles.firstAngle);
            angles_telemetry[1].setValue(angles.secondAngle);
            angles_telemetry[2].setValue(angles.thirdAngle);


        }

    }
}
