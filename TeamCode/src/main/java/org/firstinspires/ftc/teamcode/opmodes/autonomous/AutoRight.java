package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.util.Constants;

//@Autonomous(name="AutoRight")
public class AutoRight extends AutoBase {



    public void doAuto()  {
        initRobot();

        placeSample();


        driveRight(32);
        driveReverse(14);

    }

}