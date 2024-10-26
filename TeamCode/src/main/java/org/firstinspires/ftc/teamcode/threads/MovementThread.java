package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MotorRampProfile;
import org.firstinspires.ftc.teamcode.util.RobotDevices;


public class MovementThread extends RobotThread {

    private Gamepad _gamepad;
    private MecanumDrive _drive;

    private Servo _armRelease;

    private Telemetry _telemetry;

    MotorRampProfile _Joy1X, _Joy1Y, _Joy2X;
    DcMotor _hang0,_hang1;
    Telemetry.Item T_wall;

    public MovementThread(Gamepad gamepad,
                          MecanumDrive drive,
                          DcMotor hang0,
                          DcMotor hang1,
                          Servo armRelease,
                          Telemetry telemetry) {
        this._gamepad = gamepad;
        this._drive = drive;
        this._telemetry = telemetry;
        this._hang0 = hang0;
        this._hang1 = hang1;

        this._hang1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this._hang0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this._armRelease = armRelease;

        _Joy1Y = new MotorRampProfile(Constants.MecanumDrive.RAMP_RATE_J1X, Constants.MecanumDrive.RAMP_RATE_J1X/4);
        _Joy1X = new MotorRampProfile(Constants.MecanumDrive.RAMP_RATE_J1Y,Constants.MecanumDrive.RAMP_RATE_J1Y/4);
        _Joy2X = new MotorRampProfile(Constants.MecanumDrive.RAMP_RATE_J2X,Constants.MecanumDrive.RAMP_RATE_J2X/4);


    }

    private double deadzone(double power, double zone) {
        return Math.abs(power) > zone ? power : 0.0;
    }


    double fine_control = 1.0;

    public void run() {
        //Telemetry.Item hangpos0,hangpos1;
        //hangpos1 = _telemetry.addData("HP1", 0);
        //hangpos0 = _telemetry.addData("HP0", 0);

        while (!isCancelled()) {
            //hangpos1.setValue(_hang1.getCurrentPosition());
            //hangpos0.setValue(_hang0.getCurrentPosition());
            _telemetry.update();



            if (_gamepad.dpad_up) {
                _hang0.setPower(1);
                _hang1.setPower(1);
            } else if (_gamepad.dpad_down) {
                _hang0.setPower(-1);
                _hang1.setPower(-1);
            }
            else {
                _hang0.setPower(0);
                _hang1.setPower(0);
            }

            if (_gamepad.a) {
                _armRelease.setPosition(1);
            }
            else if (_gamepad.x) {
                _armRelease.setPosition(-1);
            }

            if (_gamepad.left_trigger >=0.3) {
                _drive.setBrakes();
            }
            else {
                _drive.clearBrakes();
            }


            fine_control = _gamepad.right_trigger >= 0.3 ? 1.3 - _gamepad.right_trigger : 1.0;
            _drive.moveRect(
                    _Joy1Y.newRamp(deadzone(_gamepad.left_stick_y * fine_control, Constants.MecanumDrive.ZONE_FORWARD)),
                    _Joy1X.newRamp(deadzone(_gamepad.left_stick_x * fine_control, Constants.MecanumDrive.ZONE_LATERAL)),
                    _Joy2X.newRamp(deadzone(_gamepad.right_stick_x/2 * fine_control, Constants.MecanumDrive.ZONE_ROTATION))
            );
        }
    }
}


