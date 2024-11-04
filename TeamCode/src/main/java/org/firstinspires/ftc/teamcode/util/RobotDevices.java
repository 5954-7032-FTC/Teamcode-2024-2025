package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

/**
 * RobotDevices is a singleton that holds references for all hardware.&nbsp;
* @author      Greg Weaver
* @version     %I%, %G%
* @since       1.0
*/
public class RobotDevices {

    public DcMotor [] wheels;
    public DcMotor [] lifters;
    public DcMotor hang0,hang1;
    public IMU imu;
    public CRServo[] intakeServos;
    public CRServo intakeLift;
    public CRServo intakeTilt;
    public Servo armRelease;

    public TouchSensor upperLift;

    public TouchSensor lowerLift;

    public ColorSensor intakeColorSensor;
    public DistanceSensor intakeDistanceSensor;


    protected static RobotDevices robotDevices;

    public static RobotDevices getDevices(HardwareMap hardwareMap) {
        if (robotDevices == null ) {
            robotDevices = new RobotDevices(hardwareMap);
        }
        return robotDevices;
    }

    private RobotDevices(HardwareMap hardwareMap) {
        //wheels setup
        wheels = new DcMotor[]{
                hardwareMap.dcMotor.get("D_FR"),
                hardwareMap.dcMotor.get("D_RR"),
                hardwareMap.dcMotor.get("D_RL"),
                hardwareMap.dcMotor.get("D_FL")
        };

        // imu setup
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters imuParameters;
        imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(imuParameters);


        lifters = new DcMotor[]{
                hardwareMap.dcMotor.get("LIFT0"),
                hardwareMap.dcMotor.get("LIFT1")
        };
        hang0 = hardwareMap.dcMotor.get("HANG0");
        hang1 = hardwareMap.dcMotor.get("HANG1");


       intakeServos = new CRServo[]{
               hardwareMap.crservo.get("INTAKE0"),
               hardwareMap.crservo.get("INTAKE1")
       };

        intakeLift = hardwareMap.crservo.get("INLIFT");
        intakeTilt = hardwareMap.crservo.get("INTILT");
        armRelease = hardwareMap.servo.get("RELEASE");

        upperLift = hardwareMap.touchSensor.get("UPPER");
        lowerLift = hardwareMap.touchSensor.get("LOWER");


        intakeColorSensor = hardwareMap.colorSensor.get("COLOR");
        intakeDistanceSensor = hardwareMap.get(DistanceSensor.class,"COLOR");
    }
}
