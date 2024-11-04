package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoLeft")
public class AutoLeft extends AutoBase {


    public void doAuto()  {
        initRobot();

        telemetry.update();


        placeSample();

        // now move to parking....
        driveLeft(17);
        driveForward(28);
        driveRight(12);
    }

}