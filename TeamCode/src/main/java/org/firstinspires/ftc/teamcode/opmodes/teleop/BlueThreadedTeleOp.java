package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp-Blue")
public class BlueThreadedTeleOp extends ThreadedTeleOp {
    @Override
    public Color getColor() {
        return Color.BLUE;
    }
}
