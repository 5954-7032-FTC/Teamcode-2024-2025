package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveSubsystemImpl;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveParameters;
import org.firstinspires.ftc.teamcode.threads.ArmControlThread;
import org.firstinspires.ftc.teamcode.threads.MovementThread;
import org.firstinspires.ftc.teamcode.util.*;
//import org.firstinspires.ftc.teamcode.util.Constants;


@TeleOp(name = "TeleOp")
public  class ThreadedTeleOp extends OpMode {
    ArmControlThread _arm;
    MovementThread _move;

    Telemetry.Item _threadCount;//,_bot_cone;
    RobotDevices robotDevices;


    @Override
    public void init() {
        //robotDevices = org.firstinspires.ftc.teamcode.util.RobotDevices.getDevices(hardwareMap);
        robotDevices = RobotDevices.getDevices(hardwareMap);
       /* _arm = new ArmControlThread(gamepad2,
                telemetry,
                robotDevices.intakeServos
        );
*/
        /*PixelDelivery pixelDelivery = new PixelDelivery(
                telemetry,
                robotDevices.leftPixelArm,
                robotDevices.leftPixelFlip,
                robotDevices.rightPixelArm,
                robotDevices.rightPixelFlip,
                robotDevices.sensorServos,
                robotDevices.frontSensor,
                robotDevices.rearSensor,
                robotDevices.topDropServo
        );*/

        MecanumDriveParameters driveParameters = new MecanumDriveParameters();
        driveParameters.telemetry = telemetry;
        driveParameters.motors = robotDevices.wheels;
        driveParameters.ENCODER_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.FREE_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.REVERSED_WHEELS = new int[]{0,1};
        driveParameters.imu = robotDevices.imunew;
        driveParameters.clampAngle = true;

        _move = new MovementThread(gamepad1,
                new MecanumDriveSubsystemImpl(driveParameters),
                robotDevices.hang0,
                robotDevices.hang1,
                robotDevices.armRelease,
                telemetry);

        _arm = new ArmControlThread(gamepad2,telemetry,robotDevices.lifters,
                robotDevices.intakeServos,robotDevices.intakeLift,robotDevices.intakeTilt,
                robotDevices.upperLift,robotDevices.lowerLift, robotDevices.intakeColorSensor, robotDevices.intakeDistanceSensor);

        _threadCount = telemetry.addData("Threads", Thread.activeCount());


    }

    @Override
    public void start() {
        super.start();
        _arm.start();
        _move.start();
    }

    @Override
    public void loop() {
        _threadCount.setValue(Thread.activeCount());
        telemetry.update();

    }

    @Override
    public void stop() {
        super.stop();
        _arm.cancel();
        _move.cancel();

    }

}
