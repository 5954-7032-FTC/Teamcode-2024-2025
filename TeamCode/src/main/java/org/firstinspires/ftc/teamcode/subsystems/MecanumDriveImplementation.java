package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.IntVector2d;
import org.firstinspires.ftc.teamcode.util.Pose2D;
import org.firstinspires.ftc.teamcode.util.WheelPositions;

public class MecanumDriveImplementation implements MecanumDrive {


    protected DcMotor [] _motors;
    protected Telemetry _telemetry;
    protected int [] _FREE_WHEELS; // no encoder wheels (RIGHT, LEFT)
    protected int [] _ENCODER_WHEELS; // encoder wheels (RIGHT, LEFT)
    protected int [] _REVERSED_WHEELS; // reversed motors list

    protected  int [] _lastWheelPositions;
    protected Pose2D _position = new Pose2D(0,0,0);

    protected Telemetry.Item T_FrontRightSpeed, T_FrontLeftSpeed, T_RearRightSpeed,
            T_RearLeftSpeed, T_FrontRightPosition, T_FrontLeftPosition, T_RearRightPosition, T_RearLeftPosition,
            T_POS;


    protected boolean _clampAngle = false;

    public MecanumDriveImplementation(MecanumDriveParameters parameters) {
        init(parameters.motors,parameters.telemetry,parameters.FREE_WHEELS,parameters.ENCODER_WHEELS,parameters.REVERSED_WHEELS,parameters.clampAngle);
    }


    protected void init(DcMotor[] _motors, Telemetry _telemetry,
                            int[] _FREE_WHEELS, int[] _ENCODER_WHEELS, int[] _REVERSED_WHEELS,
                            boolean clampAngle) {
        this._motors = _motors;
        this._telemetry = _telemetry;
        this._FREE_WHEELS = _FREE_WHEELS;
        this._ENCODER_WHEELS = _ENCODER_WHEELS;
        this._REVERSED_WHEELS = _REVERSED_WHEELS;
        setRunMode(_FREE_WHEELS, DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_USING_ENCODER);
        setReverseWheels(_REVERSED_WHEELS);
        //setZeroPowerBrake(new int[]{0,1,2,3});
        setZeroPowerCoast(new int[]{0,1,2,3});

        this._clampAngle = clampAngle;

        // set up telemetry objects:
        T_POS = _telemetry.addData("Field(x,y,theta):","(0,0,0)");
        T_FrontRightSpeed = _telemetry.addData("FRS","0");
        T_FrontLeftSpeed = _telemetry.addData("FLS","0");
        T_RearRightSpeed = _telemetry.addData("RRS","0");
        T_RearLeftSpeed = _telemetry.addData("RLS","0");
        T_FrontRightPosition = _telemetry.addData("FRP","0");
        T_FrontLeftPosition = _telemetry.addData("FLP","0");
        T_RearRightPosition = _telemetry.addData("RRP","0");
        T_RearLeftPosition = _telemetry.addData("RLP","0");

    }


    @Override
    public void moveRect(double forward, double lateral, double rotate) {
        //translate into polar and move accordingly.
        movePolar(Math.hypot(forward,lateral),
                Math.atan2(-forward,lateral),
                rotate);
    }

    @Override
    public void movePolar(double power, double angle, double rotate) {

        angle -= Constants.PI_OVER4;
        rotate *= Constants.ROTATION_RATE;
        double sine  = Math.sin(angle);
        double cosine = Math.cos(angle);

        //cosine = Math.sqrt(1-sine*sine);
        double scale = ( (power + Math.abs(rotate)) > 1 ) ?
                Constants.SPEED_FACTOR /(power + rotate) :
                Constants.SPEED_FACTOR /Math.sqrt(2) ;

        double [] wheelSpeeds = {
                scale * (power * sine - rotate),   // Front Right
                scale * (power * cosine - rotate), // Rear Right
                scale * (power * sine + rotate),   // Rear Left
                scale * (power * cosine + rotate)  // Front Left
        };
        setMotorSpeeds(wheelSpeeds);
        updateOdometry();
        outputTelemetry(TelemetryTypes.WHEEL_SPEEDS);
        outputTelemetry(TelemetryTypes.WHEEL_POSITIONS);
        outputTelemetry(TelemetryTypes.FIELD_POSITION);
    }


    public Pose2D get_position() {
        return _position;
    }

    public void updateOdometry() {

        if (_lastWheelPositions == null) {
            _lastWheelPositions = readEncoders();
            return;
        }
        int [] currentWheelPositions = readEncoders();
        int [] wheelMovements = new int[currentWheelPositions.length];

        for (int i=0; i< currentWheelPositions.length; i++) {
            wheelMovements[i] = currentWheelPositions[i] - _lastWheelPositions[i];
        }



        IntVector2d frontRight,frontLeft,rearLeft,rearRight;

        frontRight = new IntVector2d(-wheelMovements[WheelPositions.FrontRight.position], wheelMovements[WheelPositions.FrontRight.position]);
        frontLeft = new IntVector2d(wheelMovements[WheelPositions.FrontLeft.position], wheelMovements[WheelPositions.FrontLeft.position]);
        rearRight = new IntVector2d(wheelMovements[WheelPositions.RearRight.position], wheelMovements[WheelPositions.RearRight.position]);
        rearLeft = new IntVector2d(-wheelMovements[WheelPositions.RearLeft.position], wheelMovements[WheelPositions.RearLeft.position]);

        IntVector2d result = IntVector2d.add(frontRight,frontLeft,rearRight,rearLeft);
        _position.x += Math.cos(result.getLength()) * Constants.COUNTS_PER_INCH_FORWARD;
        _position.y += Math.sin(result.getLength()) * Constants.COUNTS_PER_INCH_FORWARD;
        _position.theta += result.getTheta();
        _lastWheelPositions = currentWheelPositions;

    }

    @Override
    public void outputTelemetry(TelemetryTypes type) {
        switch (type) {
            case WHEEL_SPEEDS:
                double [] speeds  = readSpeeds();
                T_RearLeftSpeed.setValue(speeds[WheelPositions.RearLeft.position]);
                T_RearRightSpeed.setValue(speeds[WheelPositions.RearRight.position]);
                T_FrontLeftSpeed.setValue(speeds[WheelPositions.FrontLeft.position]);
                T_FrontRightSpeed.setValue(speeds[WheelPositions.FrontRight.position]);
                break;
            case WHEEL_POSITIONS:
                int [] encoders  = readEncoders();
                T_RearLeftPosition.setValue(encoders[WheelPositions.RearLeft.position]);
                T_RearRightPosition.setValue(encoders[WheelPositions.RearRight.position]);
                T_FrontLeftPosition.setValue(encoders[WheelPositions.FrontLeft.position]);
                T_FrontRightPosition.setValue(encoders[WheelPositions.FrontRight.position]);
                break;
            case FIELD_POSITION:
                Pose2D pose = get_position();
                T_POS.setValue("(%f,%f,%f)",pose.x,pose.y,pose.theta);
                break;
        }

    }

    public int [] readEncoders() {  // return the encoder values if there are encoder motors.
        if (_ENCODER_WHEELS != null && _ENCODER_WHEELS.length > 0 ) {
            int [] encoders = new int[_ENCODER_WHEELS.length];
            for (int i=0; i<_ENCODER_WHEELS.length; i++) {
                encoders[i] = _motors[_ENCODER_WHEELS[i]].getCurrentPosition();
            }
            return encoders;
        }
        return null;
    }

    public double[] readSpeeds() {
        if (_motors != null) {
            double [] speeds = new double[_motors.length];
            for (int i=0; i< _motors.length; i++) {
                speeds[i] = _motors[i].getPower();
            }
            return speeds;
        }
        return null;
    }


    protected void setReverseWheels(int [] wheels) {
        if (wheels != null)
            for (int wheel : wheels)
                _motors[wheel].setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void setMotorSpeeds(double [] speeds) {
        _motors[0].setPower(Range.clip(speeds[0],-1,1));
        _motors[1].setPower(Range.clip(speeds[1],-1,1));
        _motors[2].setPower(Range.clip(speeds[2],-1,1));
        _motors[3].setPower(Range.clip(speeds[3],-1,1));
//        for (int i = 0; i < _motors.length; i++) _motors[i].setPower(Range.clip(speeds[i], -1, 1));
    }

    protected void setRunMode(int [] wheels, DcMotor.RunMode mode) {
        if (wheels != null)
            for (int wheel : wheels)   // for (int wheel =0; wheel< wheels.length; wheel++)
                _motors[wheel].setMode(mode);
    }

    public void setZeroPowerBrake(int [] wheels) {
        if (wheels != null)
            for (int wheel : wheels)
                _motors[wheel].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setZeroPowerCoast(int[] wheels) {
        if (wheels != null)
            for (int wheel : wheels)
                _motors[wheel].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    public void setBrakes() {
        setZeroPowerBrake(new int[]{0,1,2,3});
    }
    public void clearBrakes() {
        setZeroPowerCoast(new int[]{0,1,2,3});
    }


}
