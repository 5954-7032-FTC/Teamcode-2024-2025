package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp-Red")
public class RedThreadedTeleOp extends ThreadedTeleOp {
    @Override
    public Color getColor() {
        return Color.RED;
    }
}
