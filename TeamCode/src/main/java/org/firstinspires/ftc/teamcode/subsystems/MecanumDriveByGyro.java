package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.Constants;

public class MecanumDriveByGyro extends MecanumDriveImplementation implements DriveRobot, Gyro {

    //int [] FORWARD_VALUES, REVERSE_VALUES, LATERAL_LEFT_VALUES, LATERAL_RIGHT_VALUES, ROTATE_VALUES;

    double robotHeading;
    double driveSpeed;
    double turnSpeed;
    double headingError;
    Telemetry.Item T_angle;

    private Orientation _lastAngles = new Orientation();
    private double _currentAngle = 0.0;
    protected ImuDevice imu;
    protected boolean fixHeadingToZero = false;

    @Override
    public void resetAngle() {
        _lastAngles = imu.getOrientation();
        _currentAngle = 0.0;
    }

    @Override
    public double getAngle() {
        Orientation orientation = imu.getOrientation();
        double deltaAngle = orientation.firstAngle - _lastAngles.firstAngle;
        if (deltaAngle > 180) deltaAngle -= 360;
        if (deltaAngle <= -180) deltaAngle += 360;

        _currentAngle += deltaAngle;
        _lastAngles = orientation;
        _telemetry.addData("currentGyroZ", orientation.firstAngle);
        return _currentAngle;
    }

    @Override
    public void turn(double degrees) {

    }

    @Override
    public void turnTo(double degrees) {
        Orientation orientation = imu.getOrientation();
        double error = degrees - orientation.firstAngle;
        if (error > 180) error -= 360;
        if (error <= -180) error += 360;
        turn(error);
    }

    @Override
    public double getAbsoluteAngle() {
        return imu.getOrientation().firstAngle;
    }


    public MecanumDriveByGyro(MecanumDriveParameters parameters, ImuDevice imu) {
        super(parameters);
        //init();
        this.imu = imu;
        imu.resetHeading();

        T_angle = _telemetry.addData("Heading", "");
    }


    @Override
    public void outputTelemetry(org.firstinspires.ftc.teamcode.subsystems.TelemetryTypes type) {
        super.outputTelemetry(type);
    }

    @Override
    public void driveReverse(double distance) {
        driveRobot(distance, Constants.REVERSE_VALUES);
    }

    @Override
    public void driveRight(double distance) {
        driveRobot(distance, Constants.LATERAL_RIGHT_VALUES);
    }

    @Override
    public void driveForward(double distance) {
        driveRobot(distance, Constants.FORWARD_VALUES);
    }

    @Override
    public void driveLeft(double distance) {
        driveRobot(distance, Constants.LATERAL_LEFT_VALUES);
    }

    public void driveInit() {
        // setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void driveRobot(double inches, int[] direction) {
        // Determine new target position, and pass to motor controller
        int moveCounts = moveCounts(inches);
        int[] targets = setTargetPositions(moveCounts, direction);

        //setRunMode(_ENCODER_WHEELS, DcMotor.RunMode.RUN_TO_POSITION);
        // Set the required driving speed  (must be positive? (nope) for RUN_TO_POSITION)
        // Start driving straight, and then enter the control loop

        moveRobotDirection(Constants.DRIVE_SPEED, 0, direction);

        targetHeading = fixHeadingToZero ? 0.0 : imu.getHeading();

        // keep looping while we are still active, and BOTH motors are running.
        //while (leftIsBusy() && rightIsBusy()) {
        while (!checkTargetReached(targets, Constants.MecanumDrive.POSITION_TOLERANCE)) {

            // Determine required steering to keep on heading
            turnSpeed = direction[4] * getSteeringCorrection(robotHeading, Constants.P_DRIVE_GAIN);

            // Apply the turning correction to the current driving speed.
            moveRobotDirection(driveSpeed, turnSpeed, direction);

        }

        // Stop all motion & Turn off RUN_TO_POSITION
        stopRobot();
    }

    double targetHeading;

    @Override
    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        imu.getHeading();


        //targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        //robotHeading = getRawHeading() - headingOffset;

        // Determine the heading current error
        headingError = targetHeading - imu.getHeading();

        //headingError %= 360;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;


        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return 0.0;
        //return -Range.clip(headingError * proportionalGain, -1, 1);
    }

    @Override
    public void holdHeading(double maxTurnSpeed, double heading, double holdTime) {
        ElapsedTime holdTimer = new ElapsedTime();
        holdTimer.reset();
        // keep looping while we have time remaining.
        while (holdTimer.time() < holdTime) {
            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, Constants.P_TURN_GAIN);
            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);
            // Pivot in place by applying the turning correction
            moveRobotDirection(0, turnSpeed, Constants.REVERSE_VALUES);
        }
        // Stop all motion;
        stopRobot();
    }

    public int moveCounts(double distance) {
        return (int) (distance * Constants.COUNTS_PER_INCH_FORWARD);
    }

    public void stopRobot() {
        setMotorSpeeds(new double[]{0.0, 0.0, 0.0, 0.0});
    }

    public void moveRobotDirection(double power, double rotate, int[] direction) {
        driveSpeed = power;
        turnSpeed = rotate;
        rotate *= direction[4];
        double[] wheelSpeeds = {
                direction[0] * power - rotate,   // Front Right
                direction[1] * power - rotate, // Rear Right
                direction[2] * power + rotate,   // Rear Left
                direction[3] * power + rotate  // Front Left
        };
        setMotorSpeeds(wheelSpeeds);
        outputTelemetry(TelemetryTypes.WHEEL_POSITIONS);
        outputTelemetry(TelemetryTypes.WHEEL_SPEEDS);
        outputTelemetry(TelemetryTypes.FIELD_POSITION);
    }

    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        imu.resetHeading();
        robotHeading = 0;
    }

    public boolean checkTargetReached(int[] targets, int tolerance) {
        for (int i = 0; i < _ENCODER_WHEELS.length; i++) {
            if (Math.abs(_motors[i].getCurrentPosition() - targets[i]) <= tolerance) return true;
        }
        return false;
    }

    public int[] setTargetPositions(int target, int[] directions) {
        int[] targets = new int[4];
        for (int i = 0; i < _ENCODER_WHEELS.length; i++) {
            targets[i] = _motors[i].getCurrentPosition() + target * directions[i];
            _motors[i].setTargetPosition(targets[i]);
        }
        return targets;
    }


    // CCW == positive degrees
    // CW == negative degrees


    public void setFixHeadingToZero() {
        fixHeadingToZero = true;
    }

}
