package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoLeft")
public class AutoLeft extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initRobot();
        while (!opModeIsActive()) ;
        doAuto();
    }

    public void doAuto() throws InterruptedException {
        initRobot();

        telemetry.update();

        driveReverse(2);
        _armRelease.setPosition(1);
        //get the hang mech to work
        double start = _hang0.getCurrentPosition();
        while (_hang0.getCurrentPosition() < start+14000 ) {
            _hang1.setPower(1);
            _hang0.setPower(1);
        }
        _hang0.setPower(0);
        _hang1.setPower(0);

        while (!_armSubSystem.upperIsPressed())
            _armSubSystem.moveArm(1);
        _armSubSystem.moveArm(0);
        double start_pos = _armSubSystem.getArmPosition0();
        while (_armSubSystem.getArmPosition0() < start_pos+1500)
            _armSubSystem.moveArm(-1);
        _armSubSystem.moveArm(0);

        driveReverse(9);

        start_pos = _armSubSystem.getArmPosition0();
        while (_armSubSystem.getArmPosition0() < start_pos+1300)
            _armSubSystem.moveArm(-1);
        _armSubSystem.moveArm(0);


        driveForward(12);
        driveRight(17);
        driveReverse(28);
        driveLeft(12);
    }

}