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

    public static final String GYRO = "gyro";
    public static final String LEFT_COLOR_SENSOR = "left_color_sensor";
    public static final String RIGHT_COLOR_SENSOR = "right_color_sensor";
    public static final String LEFT_DRIVE_MOTOR = "left_drive";
    public static final String RIGHT_DRIVE_MOTOR = "right_drive";
    public static final String SWEEPER_MOTOR = "sweeper";
    public static final String PUSHER_SERVO = "pusher";
    public static final String LEFT_ARM = "leftarm";
    public static final String RIGHT_ARM = "rightarm";
    public static final String GATE_SERVO = "gate";
    public static final String FRONT_RANGE_SENSOR = "sensor_range";
    public static final String BACK_RANGE_SENSOR = "sensor2_range";
    public static final String LIGHT_SENSOR = "light_sensor";

}
