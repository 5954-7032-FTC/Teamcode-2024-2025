package org.firstinspires.ftc.teamcode.util;

public final class Constants {


    //public static final double rampRate=1.5;

    //public static final double sampleDistance=25;
    //math constants to speed up calculations
    //public static final double SQRT2=Math.sqrt(2);

//public static final double DEAD_ZONE_CLAMP_ANGLE =  Math.toRadians(10);
    public static final double PI_OVER4=Math.PI/4;
    public static final double Y_DISTANCE_RATIO = 12.0/13.0;
    public static final double X_DISTANCE_RATIO = 30.0/26.0;
    // motor constants based on physical properties of the robot
    public static final double COUNTS_PER_MOTOR_REV = 1120;   // 1120 per revolution
    public static final double DRIVE_GEAR_REDUCTION = 0.75; //   3/4
    public static final double WHEEL_DIAMETER_INCHES = 96/25.4;     // For figuring circumference
    public static final double COUNTS_PER_INCH_FORWARD = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final int [] REVERSE_VALUES = new int[]{ 1, 1, 1, 1,1};
    public static final int [] FORWARD_VALUES = new int[]{-1, -1, -1, -1,-1};
    public static final int [] LATERAL_RIGHT_VALUES = new int[]{1,-1,1,-1,1};
    public static final int [] LATERAL_LEFT_VALUES = new int[]{-1,1,-1,1,1};
    public static final int [] ROTATE_VALUES = new int[]{1,1,-1,-1};

    public static final double SPEED_FACTOR =2.0;
    public static final double ROTATION_RATE =1.0;


    // speed settings
    public static final double     DRIVE_SPEED             = 0.4;     // Max driving speed for better distance accuracy.
    //public static final double     TURN_SPEED              = 0.2;     // Max Turn speed to limit turn rate
    public static final double     HEADING_THRESHOLD       = 1 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.
    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not corrects strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value
    public static final double     P_TURN_GAIN            = 0.02;     // Larger is more responsive, but also less stable
    //maybe only use one of these.
    public static final double     P_DRIVE_GAIN           = 0.025;     // Larger is more responsive, but also less stable


    //public final static String VuforiaKey = "Ac6tBZr/////AAABmd2+0ZS1DUaxvjeLOQXt6BocTj8MS8ZdGc3iaWgJcb4x+GTRiMydjRed7kvoAvq0x21glktV2ekv6Nq8WLNelf5Chl5vN4X9QjUKYvH1fgh72q2cY2w5lMO5tmoOAbyNlN4hSM+RdaWXC7MpY95EVbwz584eP2KUQ97DMCFYqGj6zaVTap2FQ/U2rK7XDNp+s0mdm1+2dvJh6bw0Xpp/DjkUG7RB3uLZe0niObsnONPJg29RCf2eOVY/NP7qjXZamhGLjR1Cpj+U2HGh5DIqCauT/lvn/PDfa+H8ErXG0grgeSqQUHGYlsnYiYrp7Q70RKeebAeOsMVVj6zNhjI6dGE06u3JZgT6aF5EMxnJyc2X";

    // Arm control dead zone section
    public static final double armControlDeadzone = 0.2;
    public static final int HANG_MOVE_DISTANCE = 14000;
    public static final int SPECIMEN_PREPARE_POSITION = -2900;
    public static final int SPECIMEN_PLACEMENT_POSITION_END = 1300;
    public static final class MecanumDrive {
        public static final double ZONE_LATERAL   = 0.2;
        public static final double ZONE_FORWARD   = 0.2;
        public static final double ZONE_ROTATION  = 0.1;
        public static final double RAMP_RATE_J1X  = 1.5;
        public static final double RAMP_RATE_J1Y  = 1.5;
        public static final double RAMP_RATE_J2X  =  1.5;
        public static final int POSITION_TOLERANCE =10;
        //public static final double FINE_CONTROL   = 0.35;
    }
}
