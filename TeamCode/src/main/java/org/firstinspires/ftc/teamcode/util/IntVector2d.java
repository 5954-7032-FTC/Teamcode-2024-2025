package org.firstinspires.ftc.teamcode.util;

public class IntVector2d {
    protected int _x,_y;
    public IntVector2d(int x, int y) {
        _x = x;
        _y = y;
    }
    public int getX() {return _x;};
    public int getY() {return _y;};
    public IntVector2d add(IntVector2d v) {
        return sum(this,v);
    }

    public static IntVector2d add ( IntVector2d ... values) {
        int newx=0, newy=0;
        for (int i=0; i<values.length; i++) {
            newx += values[i]._x;
            newy += values[i]._y;
        }
        return new IntVector2d(newx,newy);

    }

    public double getLength() {
            return Math.sqrt(1.0*(_x*_x + _y*_y));
        }

    public double getTheta() {
        return Math.atan2(1.0*_y,1.0*_x);
    }

    public static IntVector2d sum(IntVector2d u, IntVector2d v) {
        return new IntVector2d( u._x + v._x, u._y + v._y);
    }
}