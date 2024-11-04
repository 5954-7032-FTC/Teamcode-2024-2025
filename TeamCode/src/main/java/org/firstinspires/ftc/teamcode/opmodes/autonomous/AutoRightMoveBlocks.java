package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoRightWithBlocks")
public class AutoRightMoveBlocks extends AutoBase {



    public void doAuto()  {
        initRobot();

        placeSample();


        driveRight(18);
        driveForward(22);


        // we are in front of blocks
        driveRight(8);
        driveReverse(36);
        driveForward(28);
        driveRight(7);
        driveReverse(30);
        driveForward(28);
        driveRight(7);
        driveReverse(30);


    }

}