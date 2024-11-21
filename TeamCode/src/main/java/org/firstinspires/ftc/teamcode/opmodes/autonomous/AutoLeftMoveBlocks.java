package org.firstinspires.ftc.teamcode.opmodes.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoLeftWithBlocks")
public class AutoLeftMoveBlocks extends AutoBase {



    public void doAuto()  {
        initRobot();

        placeSample();


        driveLeft(18);
        driveForward(22);
        pauseMillis(500);


        // we are in front of blocks
        driveLeft(4);
        driveRotate(1.75);
        driveReverse(36);
        pauseMillis(500);
        driveForward(28);
        pauseMillis(500);
        //driveAntiRotate(-1);
        driveLeft(5);
        driveReverse(32);
        //driveForward(28);
        //pauseMillis(500);
        //driveLeft(6);
        //driveReverse(30);
        driveForward(30);
        driveRight(28);


    }

}