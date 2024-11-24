package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubSystem;
import org.firstinspires.ftc.teamcode.subsystems.ImuDevice;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveByGyro;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDriveParameters;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.RobotDevices;

public abstract class AutoBase extends LinearOpMode {

    protected RobotDevices robotDevices;
    protected MecanumDriveByGyro _move;
    protected ArmSubSystem _armSubSystem;
    protected Servo _armRelease;
    protected DcMotor _hang0,_hang1;
    protected Telemetry.Item T_IMU;
    protected ImuDevice _imu;


    public void initRobot() {
        telemetry.setAutoClear(false);

        robotDevices = RobotDevices.getDevices(hardwareMap);

        _imu =  new ImuDevice(robotDevices.imu);
        T_IMU = telemetry.addData("IMU", "(Yaw:(%f)",
                _imu.getHeading()
        );

        MecanumDriveParameters driveParameters = new MecanumDriveParameters();
        driveParameters.motors = robotDevices.wheels;
        driveParameters.ENCODER_WHEELS = new int[]{0, 1, 2, 3};
        driveParameters.REVERSED_WHEELS = new int[]{2, 3};
        driveParameters.telemetry = telemetry;
        _move = new MecanumDriveByGyro(driveParameters, new ImuDevice(robotDevices.imu));
        _move.setFixHeadingToZero();

        _move.resetHeading();

        _armSubSystem = new ArmSubSystem(robotDevices.lifters,
                robotDevices.intakeServos,robotDevices.intakeLift,robotDevices.intakeTilt,
                robotDevices.upperLift,robotDevices.lowerLift, robotDevices.intakeColorSensor,
                robotDevices.intakeDistanceSensor);

        _armRelease = robotDevices.armRelease;
        _hang0 = robotDevices.hang0;
        _hang1 = robotDevices.hang1;
        _move.driveInit();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        initRobot();
        waitForStart();
        doAuto();
    }


    public abstract void doAuto();

    public void pauseMillis(long millis) {
        try {
            Thread.sleep(millis);
        }
        catch (InterruptedException ignored) {
        }
    }

    public void pauseSeconds(int seconds) {
        pauseMillis((long)seconds * 1000);
    }

    public void driveRotate(double distanceInches) {
        _move.driveRotate(distanceInches);
    }

    public void driveAntiRotate(double distanceInches) {
        _move.driveRotate(distanceInches);
    }

    public void driveReverse(double distanceInches) {
        _move.driveReverse(distanceInches*Constants.Y_DISTANCE_RATIO);

        T_IMU.setValue(_imu.getHeading());
        telemetry.update();
    }

    public void driveRight(double distanceInches) {
        _move.driveRight(distanceInches*Constants.X_DISTANCE_RATIO);

        T_IMU.setValue(_imu.getHeading());
        telemetry.update();
    }

    public void driveForward(double distanceInches) {
        _move.driveForward(distanceInches*Constants.Y_DISTANCE_RATIO);

        T_IMU.setValue(_imu.getHeading());
        telemetry.update();
    }

    public void driveLeft(double distanceInches) {
        _move.driveLeft(distanceInches*Constants.X_DISTANCE_RATIO);

        T_IMU.setValue(_imu.getHeading());
        telemetry.update();
    }


    public void placeSample() {
        driveForward(2);               // move from wall
        releaseArm();                 // release the arm
        moveHangToRelativePosition(Constants.HANG_MOVE_DISTANCE);  // puts it at top
        //moveArmToTop();
        moveArmUpToRelativePosition(Constants.SPECIMEN_PREPARE_POSITION);   // position for placement
        driveForward(9);              // drive forward
        pauseMillis(500);
        moveArmDownToRelativePosition(Constants.SPECIMEN_PLACEMENT_POSITION_END);  // place specimen
        driveReverse(4);            // clear submersible
        moveArmToBottom();
    }

    protected void releaseArm() {
        _armRelease.setPosition(1);
    }


    protected void moveArmUpToRelativePosition(double distance) {
        double start_pos = _armSubSystem.getArmPosition0();
        while (_armSubSystem.getArmPosition0() > start_pos +distance)
            _armSubSystem.moveArm(1.0);
        _armSubSystem.moveArm(0);

    }
    protected void moveArmDownToRelativePosition(double distance) {
        double start_pos = _armSubSystem.getArmPosition0();
        while (_armSubSystem.getArmPosition0() < start_pos +distance)
            _armSubSystem.moveArm(-1.0);
        _armSubSystem.moveArm(0);

    }
    protected void moveArmToRelativePosition(double distance) {
        double start_pos = _armSubSystem.getArmPosition0();
        while (_armSubSystem.getArmPosition0() < start_pos +distance)
            _armSubSystem.moveArm(-1);
        _armSubSystem.moveArm(0);
    }

    protected void moveArmToTop() {
        while (!_armSubSystem.upperIsPressed())
            _armSubSystem.moveArm(1);
        _armSubSystem.moveArm(0);
    }

    protected void moveArmToBottom() {
        while (!_armSubSystem.lowerIsPressed()) {
            _armSubSystem.moveArm(-1);
        }
        _armSubSystem.moveArm(0);
    }

    protected void moveHangToRelativePosition(double distance) {
        double start = _hang0.getCurrentPosition();
        while (_hang0.getCurrentPosition() < start +distance ) {
            _hang1.setPower(-1);
            _hang0.setPower(1);
        }
        _hang0.setPower(0);
        _hang1.setPower(0);
    }

}