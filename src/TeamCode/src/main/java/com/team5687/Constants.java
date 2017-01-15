package com.team5687;

/**
 * Created by stephen on 23/10/16.
 */

public class Constants {

    public static final double     COUNTS_PER_MOTOR_REV    = 1120 ;    // eg: TETRIX Motor Encoder
    public static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    public static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);


    // If we read a value higher than this from the ground sensor, the robot will think it's found the line
    // (may need to do some calibration if this is inaccurate)
    public static final double LINE_DETECTION_MINIMUM  = 0.4;

    public static final double PUSHER_SERVO_MAX = 0.6;
    public static final double PUSHER_SERVO_MIN = 0.1;

    public static final String GYRO = "gyro";
    public static final String LEFT_COLOR_SENSOR = "leftcolor";
    public static final String RIGHT_COLOR_SENSOR = "rightcolor";
    public static final String LEFT_DRIVE_MOTOR = "right_drive"; // for some reason these are backwards
    public static final String RIGHT_DRIVE_MOTOR = "left_drive";
    public static final String LEFT_LIFT_MOTOR = "left_lift";
    public static final String RIGHT_LIFT_MOTOR = "right_lift";
    public static final String SWEEPER_MOTOR = "sweeper";
    public static final String PUSHER_SERVO = "pusher";
    public static final String LEFT_ARM = "leftarm";
    public static final String RIGHT_ARM = "rightarm";
    public static final String GATE_SERVO = "gate";
    public static final String FRONT_RANGE_SENSOR = "sensor_range";
    public static final String BACK_RANGE_SENSOR = "sensor2_range";
    public static final String LIGHT_SENSOR = "sensor_range";
    public static final String DISTANCE = "distance";

}
