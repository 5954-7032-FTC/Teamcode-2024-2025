package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.TeamColor;

public class ArmSubSystem implements SubSystem {
    private final DcMotor[]  _liftMotors;

    private final CRServo [] _intakeServos;

    private final CRServo _intakeLift;

    private final CRServo _intakeTilt;

    private final TouchSensor _upperLimit;
    private final TouchSensor _lowerLimit;

    private final ColorSensor _colorSensor;

    private final DistanceSensor _distanceSensor;

    private TeamColor _teamColor;

    public ArmSubSystem(DcMotor [] liftMotors,
                        CRServo [] intakeServos,
                        CRServo intakeLift,
                        CRServo intakeTilt,
                        TouchSensor upperLimit,
                        TouchSensor lowerLimit,
                        ColorSensor colorSensor,
                        DistanceSensor distanceSensor
                        ) {
        this._liftMotors = liftMotors;
        this._intakeServos = intakeServos;
        this._intakeTilt = intakeTilt;
        this._intakeLift = intakeLift;
        this._lowerLimit = lowerLimit;
        this._upperLimit = upperLimit;
        this._distanceSensor = distanceSensor;
        this._colorSensor = colorSensor;

        this.init();
    }

    private void init() {
        _liftMotors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        _liftMotors[0].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _liftMotors[1].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        _liftMotors[0].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _liftMotors[1].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setColorToLookFor(TeamColor teamColor) {
        this._teamColor = teamColor;

    }

    public double getDistanceSensor() {
        return _distanceSensor.getDistance(DistanceUnit.MM);
    }

    public int getColorSensorBlue() {
        return _colorSensor.blue();
    }

    public int getColorSensorRed() {
        return _colorSensor.red();
    }

    public int getColorSensorGreen() {
        return _colorSensor.green();
    }

    public void moveIntakeTilt(double power) {
        _intakeTilt.setPower(power * .9);
    }

    public void moveIntakeLift(double power) {

        _intakeLift.setPower(power*.8);
    }

    public boolean upperIsPressed() {
        return _upperLimit.isPressed();
    }

    public boolean lowerIsPressed() {
        return _lowerLimit.isPressed();
    }

    public void moveArmMillis(boolean up,long millis) {
        moveArm(up?-1:1);
        try {
            Thread.sleep(millis);
        }
        catch (InterruptedException ignored) {
        }
        moveArm(0);
    }

    public void moveArm(double power) {
        if ( ( _upperLimit.isPressed() && power>=0) || (_lowerLimit.isPressed() && power<=0) )
            return;

        for (DcMotor motor : _liftMotors) {
            motor.setPower(power);
        }
    }

    public void intake(double power) {
        _intakeServos[0].setPower(power);
        _intakeServos[1].setPower(-power);
    }

    public double getArmPosition1() {
        return _liftMotors[1].getCurrentPosition();
    }
    public double getArmPosition0() {
        return _liftMotors[1].getCurrentPosition();
    }

}
