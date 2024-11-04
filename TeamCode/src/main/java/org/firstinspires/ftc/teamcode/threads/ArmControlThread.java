package org.firstinspires.ftc.teamcode.threads;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.teleop.ThreadedTeleOp;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubSystem;
import org.firstinspires.ftc.teamcode.util.Constants;

public class ArmControlThread extends RobotThread {

    private final ArmSubSystem _armSubSystem;
    private final Gamepad _gamepad;
    private final Telemetry.Item _T_colorRed;
    private final Telemetry.Item _T_colorBlue;
    private final Telemetry.Item _T_colorGreen;
    private final Telemetry.Item _T_distance;
    private final Telemetry.Item _T_state;
    private final Telemetry.Item _T_colorDetected;
    private final Telemetry.Item _T_armPosition0;
    private final Telemetry.Item _T_armPosition1;

    private final ThreadedTeleOp.Color _color;


    public ArmControlThread(Gamepad gamepad, Telemetry telemetry,
                            DcMotor [] liftMotors,
                            CRServo [] intake,
                            CRServo intakeLift,
                            CRServo intakeTilt,
                            TouchSensor upperLift,
                            TouchSensor lowerLift,
                            ColorSensor colorSensor,
                            DistanceSensor distanceSensor,
                            ThreadedTeleOp.Color color) {
        _armSubSystem = new ArmSubSystem(liftMotors,intake,intakeLift,intakeTilt,
                                         upperLift, lowerLift,colorSensor,distanceSensor);

        _gamepad = gamepad;

        //private final MotorRampProfile _Joy2Y;  //_Joy2X
        _color = color;


        _T_colorRed = telemetry.addData("COLOR_R", 0.0);
        _T_colorGreen = telemetry.addData("COLOR_G", 0.0);
        _T_colorBlue = telemetry.addData("COLOR_B", 0.0);
        _T_distance = telemetry.addData("DISTANCE", 0.0);
        _T_state = telemetry.addData("State", "EMPTY");
        _T_colorDetected = telemetry.addData("Block", "NONE");
        _T_armPosition1 = telemetry.addData("ArmPos1", 0.0);
        _T_armPosition0 = telemetry.addData("ArmPos0", 0.0);
    }


    public enum State {
        EMPTY,
        SETUP,
        WAIT,
        FULL
    }
    State CurrentState = State.EMPTY;
    @Override
    public void run() {
        double last_time=0;
        while (!isCancelled()) {

            _T_distance.setValue(_armSubSystem.getDistanceSensor());
            _T_colorBlue.setValue(_armSubSystem.getColorSensorBlue());
            _T_colorRed.setValue(_armSubSystem.getColorSensorRed());
            _T_colorGreen.setValue(_armSubSystem.getColorSensorGreen());
            _T_armPosition1.setValue(_armSubSystem.getArmPosition1());
            _T_armPosition0.setValue(_armSubSystem.getArmPosition0());


            if (isBlue())
                _T_colorDetected.setValue("BLUE");
            else if (isRed())
                _T_colorDetected.setValue("RED");
            else if (isYellow())
                _T_colorDetected.setValue("YELLOW");
            else
                _T_colorDetected.setValue("NONE");



            // Arm up and down
            if (Math.abs(_gamepad.right_stick_y) > Constants.armControlDeadzone) {
                _armSubSystem.moveArm(-(_gamepad.right_stick_y*.8));
            }
            else {
                _armSubSystem.moveArm(0);
            }


            //intake lift
            if (_gamepad.dpad_up) {
                _armSubSystem.moveIntakeLift(-1);
            }
            else if (_gamepad.dpad_down) {
                _armSubSystem.moveIntakeLift(1);
            }
            else {
                _armSubSystem.moveIntakeLift(0);
            }


            // lift tilt

            if (_gamepad.left_stick_y > 0.8)
                _armSubSystem.moveIntakeTilt(-0.8);
            else if (_gamepad.left_stick_y < -0.8)
                _armSubSystem.moveIntakeTilt(0.8);
            else
                _armSubSystem.moveIntakeTilt(-_gamepad.left_stick_y);
            /*
            if ( Math.abs(_gamepad.left_stick_y) >Constants.armControlDeadzone) {
                _armSubSystem.moveIntakeTilt(-_gamepad.left_stick_y);
            }
            else {
                _armSubSystem.moveIntakeTilt(0);
            }
             */



            
            switch (CurrentState) {
                case EMPTY:
                    _T_state.setValue("EMPTY");
                    //if distance < 20mm, and color is correct, move to SETUP, set time
                    if ((_armSubSystem.getDistanceSensor() < 20) && getColor()) {
                        new Thread(){
                            @Override
                            public void run() {
                                super.run();
                                long end = System.currentTimeMillis() +200;
                                while (System.currentTimeMillis() < end) _armSubSystem.intake(1);//do nothing
                                _armSubSystem.intake(0);
                            }
                        }.start();
                        CurrentState = State.SETUP;
                    }
                    do_triggers();
                    break;
                case WAIT:
                    _T_state.setValue("WAIT");
                    // if 500ms elapsed, then move to FULL
                    if (last_time+1500 < System.currentTimeMillis()) {
                        CurrentState = State.FULL;
                    }
                    _armSubSystem.intake(0);
                    break;
                case SETUP:
                    _T_state.setValue("SETUP");
                    //_armSubSystem.intake(0);
                    // track when we grabbed one
                    // move arm tilt up for .2 seconds.
                    last_time = System.currentTimeMillis();
                    new Thread() {
                        @Override
                        public void run() {
                            super.run();
                            long end = System.currentTimeMillis() + 350;
                            while (System.currentTimeMillis() < end) _armSubSystem.moveIntakeTilt(-.8);
                            _armSubSystem.moveIntakeTilt(0);
                        }
                    }.start();
                    // fall through to WAIT
                    CurrentState = State.WAIT;
                    break;
                case FULL:
                    _T_state.setValue("FULL");
                    // check distance > 20mm then go to EMPTY
                    if (_armSubSystem.getDistanceSensor()>20) CurrentState=State.EMPTY;
                    // triggers
                    do_triggers();
                    break;
            }

        }

    }


    private boolean getColor() {
        if (_color == ThreadedTeleOp.Color.BLUE)
            return isBlue() || isYellow();
        if (_color == ThreadedTeleOp.Color.RED)
            return isBlue() || isYellow();
        return false;
    }
    private void do_triggers() {
        if (_gamepad.right_trigger > .2) {
            _armSubSystem.intake(1);
        } else if (_gamepad.left_trigger > .2) {
            _armSubSystem.intake(-1);
        } else {
            _armSubSystem.intake(0);
        }
    }

    public boolean isYellow() {
        return (_armSubSystem.getColorSensorRed()>800 && _armSubSystem.getColorSensorGreen()>800 && _armSubSystem.getColorSensorBlue()<400);
    }

    public boolean isRed() {
        return (_armSubSystem.getColorSensorBlue() < 400 && _armSubSystem.getColorSensorRed()>500 && _armSubSystem.getColorSensorGreen()<500);
    }

    public boolean isBlue() {
        return (_armSubSystem.getColorSensorRed() < 400 && _armSubSystem.getColorSensorBlue()>500 && _armSubSystem.getColorSensorGreen()<500);
    }

}
