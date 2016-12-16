package com.team5687.opmodes;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.team5687.Constants;
import com.team5687.helpers.Logger;
import com.team5687.primitives.Motor;

import org.firstinspires.ftc.robotcore.external.Const;

import static java.lang.Thread.sleep;

public class BaseBeaconAutonmous extends OpMode {
    public enum AllianceColor {
        RED,
        BLUE
    }

    public enum State {
        // Starts here, robot is stopped
        START,
        // Moving from start to the left or right based on color to the first line
        MOVE_TO_FIRST_BEACON_LINE,
        // Hit the first line, now moving down the line towards the wall
        MOVING_DOWN_FIRST_BEACON_LINE,
        // hit the wall, pressing the beacon until it's our color
        PRESSING_FIRST_BEACON,
        // first beacon has been pressed, moving to the second line
        MOVING_TO_SECOND_BEACON_LINE,
        // moving down the second line towards the wall
        MOVING_DOWN_SECOND_BEACON_LINE,
        // pressing second beacon until it's our color
        PRESSING_SECOND_BEACON,
        // finished both beacons, moving back to center line (red / blue lines)
        MOVING_TO_CENTER_LINE,
        // Moving towards the center plate, knocking the ball off and stopping on the pad
        MOVING_TO_CENTER_BALL
    }
    //region Hardware Fields
    private HiTechnicNxtGyroSensor _gyro;
    private LightSensor _lightSensor;
    private ColorSensor _leftColorSensor;
    private ColorSensor _rightColorSensor;
    private Motor _left;
    private Motor _right;
    //endregion

    private State _currentState = State.START;

    private AllianceColor _color;

    protected BaseBeaconAutonmous(AllianceColor color) {
        _color = color;
        Logger.getInstance().SetTelemetry(telemetry);
    }

    @Override
    public void init() {
        SetupMotors();
        SetupGyro();
        //SetupLightSensor();
        //SetupColorSensor();
    }

    @Override
    public void loop() {
        double heading = 0;              // Gyro integrated heading
        // get the heading info.
        // the Modern Robotics' gyro sensor keeps
        // track of the current heading for the Z axis only.
        heading = _gyro.getRotationFraction();
        Logger.getInstance().WriteMessage(_gyro.toString());
        telemetry.update();
    }

//region Setup Code for Motors, Gryo, Light Sensor, Color Sensor, Ultrasonic

    private void SetupColorSensor() {
        _leftColorSensor = hardwareMap.colorSensor.get(Constants.LEFT_COLOR_SENSOR);
        _rightColorSensor = hardwareMap.colorSensor.get(Constants.RIGHT_COLOR_SENSOR);
    }

    private void SetupLightSensor() {
        _lightSensor = hardwareMap.lightSensor.get(Constants.LIGHT_SENSOR);
    }

    private void SetupGyro() {
        _gyro = (HiTechnicNxtGyroSensor) hardwareMap.gyroSensor.get(Constants.GYRO);
        Logger.getInstance().WriteMessage(_gyro.status());
    }

    private void SetupMotors() {
        _left = new Motor(DcMotorSimple.Direction.REVERSE, hardwareMap.dcMotor.get(Constants.LEFT_DRIVE_MOTOR), true);
        _right = new Motor(DcMotorSimple.Direction.REVERSE, hardwareMap.dcMotor.get(Constants.RIGHT_DRIVE_MOTOR), true);

        _right.SetEncoderDirection(DcMotorSimple.Direction.REVERSE);
        _left.SetEncoderDirection(DcMotorSimple.Direction.FORWARD);

        _left.Stop();
        _right.Stop();
    }

//endregion
}
