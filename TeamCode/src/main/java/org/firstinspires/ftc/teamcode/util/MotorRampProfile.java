package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotorRampProfile {
    ElapsedTime rampTimer = new ElapsedTime();
    double curPoint = 0;
    double prevT = 0;
    double prevSign = 0;
    double rampRateUp;
    double rampRateDown;


    public MotorRampProfile(double rampRateUp, double rampRateDown) {
        this.rampRateUp = rampRateUp;
        this.rampRateDown = rampRateDown;
    }

    public MotorRampProfile(double rampRate){
        this.rampRateUp = rampRate;
    }

    public double ramp(double input){
        double curTime = rampTimer.seconds();
        double curSign = Math.signum(input);
        double nextPoint = (Math.abs(curPoint) + this.rampRateUp * (curTime - prevT));
        if (prevSign != curSign && prevSign !=0){
            curPoint = 0;
        }
        else if (Math.abs(input) - Math.abs(nextPoint) > 0){
            curPoint = curSign * nextPoint;
        }
        else{
            curPoint = input;
        }
        prevSign = curSign;
        prevT = curTime;
        return curPoint;
    }


    public double newRamp(double input) {
        double curTime = rampTimer.seconds();
        double curSign = Math.signum(input);
        double rampRate = (Math.abs(curPoint) < Math.abs(input)) ? rampRateUp : rampRateDown;
        double nextPoint = (Math.abs(curPoint) + rampRate * (curTime - prevT));

        // If changing direction, allow ramping instead of resetting to 0
        if (prevSign != curSign && prevSign != 0) {
            // Gradually ramp down to 0 before ramping up again
            curPoint = curPoint - rampRateDown * (curTime - prevT);
        }
        // Gradually increase or decrease the current point
        else if (Math.abs(input) - Math.abs(nextPoint) > 0) {
            curPoint = curSign * nextPoint;
        }
        else {
            curPoint = input;
        }

        prevSign = curSign;
        prevT = curTime;
        return curPoint;
    }
}
