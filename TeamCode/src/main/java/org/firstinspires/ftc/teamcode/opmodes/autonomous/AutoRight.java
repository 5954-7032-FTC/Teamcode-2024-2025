package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoRight")
public class AutoRight extends AutoBase {

    @Override
    public void runOpMode() throws InterruptedException {
        super.runOpMode();
        initRobot();
        while (!opModeIsActive()) ;
        doAuto();
    }

    public void doAuto() throws InterruptedException {
        initRobot();


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
        while (_armSubSystem.lowerIsPressed()) {
            _armSubSystem.moveArm(-1);
        }
        _armSubSystem.moveArm(0);
        driveLeft(32);
        driveForward(6);
        /*
        telemetry.update();
        _pixelDeliveryThread.start();
        waitForStart();


        //_pixelDelivery.extendSensors();

        // drive almost to the pieces
        moveDirection(25); //25
        // start looking for a piece front or back

        _pixelDelivery.extendSensors();
        //magic
        _pixelDeliveryThread.setLooking(true);
        moveDirection(12); //12
        _pixelDeliveryThread.setLooking(false);
        // see if it saw something?
        _pixelDelivery.retractSensors();
        Constants.piecePositions position = _pixelDeliveryThread.getPosition();
        Telemetry.Item T_where = telemetry.addData("Where?", "none");

        //now move back to placement location
        moveAntiDirection(18);
        switch (position) {
            case FRONT:
                T_where.setValue("front(left side)");
                if (direction == Direction.RIGHT) {
                    _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_FRONT);
                }
                else {
                    _pixelDelivery.leftPixelDrop(Constants.pixelDropPositions.LEFT_FRONT);
                }
                //moveDirection(6);
                break;
            case REAR:
                T_where.setValue("rear(right side)");
                if (direction == Direction.RIGHT) {
                    _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_REAR);
                }
                else {
                    _pixelDelivery.leftPixelDrop(Constants.pixelDropPositions.LEFT_REAR);
                }
                //_pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_REAR);
                //moveAntiDirection(4);
                break;
            case CENTER:
                T_where.setValue("center(center)");
                if (direction == Direction.RIGHT) {
                    _pixelDelivery.rightPixelDrop(Constants.pixelDropPositions.RIGHT_CENTER);
                }
                else {
                    _pixelDelivery.leftPixelDrop(Constants.pixelDropPositions.LEFT_CENTER);
                }
                break;
        }
        telemetry.update();

        // put arm back and wait for it.....
        try { Thread.sleep(250);} catch (Exception nope) {}
        if (direction==Direction.RIGHT) {
            _pixelDelivery.rightPixelReset();
        }
        else {
            _pixelDelivery.leftPixelReset();
        }
        try { Thread.sleep(500);} catch (Exception nope) {}
        _pixelDelivery.retractSensors();
        telemetry.update();

        moveAntiDirection(15);
        _armSubSystem.moveArmMillis(true,500);
        _armSubSystem.moveArmMillis(false, 1000);
        // now wait 4 seconds
        pauseSeconds(4);

        //now place next pixel
        driveReverse(78);
        if (direction == Direction.RIGHT) {
            moveDirection(24);
        }
        else {
            moveDirection(14);
        }
        switch (position) {
            case FRONT:
                moveDirection(12);
                break;
            case REAR:
                break;
            case CENTER:
                moveDirection(6);
        }
        driveReverse(20);
        _move.holdHeading(Constants.DRIVE_SPEED,0,1);
        placePixel();
        try { Thread.sleep(250); } catch (InterruptedException ignore) {}
        unPlacePixel();
        try {
            Thread.sleep(250);
        }
        catch (InterruptedException ignore) {}


         */
    }

}