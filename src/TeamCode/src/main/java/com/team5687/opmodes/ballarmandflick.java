package com.team5687.opmodes;

import com.qualcomm.hardware.hitechnic.HiTechnicNxtGyroSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.qualcomm.robotcore.hardware.configuration.Utility;
import com.team5687.Constants;
import com.team5687.helpers.GeneralHelpers;
import com.team5687.helpers.Logger;
import com.team5687.primitives.CompassAndroid;
import com.team5687.primitives.Motor;
import com.team5687.controllers.FlipperController;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptNullOp;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static java.lang.Thread.sleep;

public class ballarmandflick extends OpMode {
    public enum AllianceColor {
        Left,
        Right
    }

    public enum BeaconSide {
        LEFT, RIGHT
    }

    public enum State {
        // Starts here, robot is stopped
        START,
        // Moving from start to the left or right based on color to the first line
        MOVE_TO_LAUNCH,
        // Hit the first line, now moving down the line towards the wall
        LAUNCH,
        // hit the wall, pressing the beacon until it's our color
        MOVE_TO_BALL,
        // Moving towards the center plate, knocking the ball off and stopping on the pad

        STOPPED
    }

    private int _generalCounter;
    private int _generalCounter2;
    //region Hardware Fields
    private HiTechnicNxtGyroSensor _gyro;
    private LightSensor _frontLightSensor;
    private LightSensor _backLightSensor;
    private ColorSensor _leftColorSensor;
    private UltrasonicSensor _ultrasonic;
    //private UltrasonicSensor _ultrasonicLeft;
    private Motor _left;
    private Motor _right;
    private Motor  _flipper;
    private Motor _sweeper;
    private Servo _pusherServer;
    private CompassAndroid _compass;
    FlipperController flipper = new FlipperController();
    //endregion

    private State _currentState = State.START;

    private AllianceColor _color;
    private BeaconSide _side;

    protected ballarmandflick(AllianceColor color, BeaconSide side) {
        _color = color;
        _side = side;
        Logger.getInstance().SetTelemetry(telemetry);
    }

    @Override
    public void init() {
        _compass = new CompassAndroid(hardwareMap.appContext);
        _compass.Init();
        Logger.getInstance().WriteMessage(hardwareMap.appContext == null ? "No App Context" : "Got an app context");
        SetupMotors();
        SetupServos();
        //SetupGyro();
        SetupLightSensor();
        SetupColorSensor();
        SetupUltrasonic();
        flipper.Init(hardwareMap);
        _currentState = State.START;
    }

    @Override
    public void loop() {


        // Code to turn the robot LEFT, just an example
//        if(!_temp) {
//            _temp = true;
//
//            double ticks = GeneralHelpers.CalculateDistanceEncode(100);
//            double leftTicksPerSecond = (ticks / 5);
//            double rightTicksPerSecond = (ticks / 10);
//            double ratio = rightTicksPerSecond / leftTicksPerSecond;
//
//            _left.SetTargetEncoderPosition((int)leftTicksPerSecond, ticks);
//            _right.SetTargetEncoderPosition((int)rightTicksPerSecond, ticks * ratio);
//            _left.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
//            _right.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }

        // USE THIS CODE TO CALIBRATE THE LINE SENSOR< YOU WANT THE getLightDetected() VALUE
//        String message = String.format("%d.%d.%d.%d, %d.%d.%d.%d, %.2f %.2f",
//                _leftColorSensor.red(),
//                _leftColorSensor.green(),
//                _leftColorSensor.blue(),
//                _leftColorSensor.alpha(),
//                _rightColorSensor.red(),
//                _rightColorSensor.green(),
//                _rightColorSensor.blue(),
//                _rightColorSensor.alpha(),
//                _lightSensor.getLightDetected(),
//                _lightSensor.getRawLightDetected());
        //Logger.getInstance().WriteMessage(message);

//        for(int i = 0; i < 3; i++)
//            telemetry.addData(Integer.toString(i), "%.2f", _compass._values[i]);
//
//        telemetry.addData("State", GetStateName(_currentState));
//        telemetry.update();


        DoState(_currentState);
    }

    @Override
    public void stop()
    {
        _compass.Stop();
    }


    private void LogNotImplementedState(State state)
    {
        Logger.getInstance().WriteMessage("NO METHOD FOR STATE:" + GetStateName(state));
    }

    //region States

    // This method is used to determine the next state
    private State GetNextState(State current)
    {
        _generalCounter = 0;
        _generalCounter2 = 0;
        switch(current)
        {
            // Starts here, robot is stopped
            case START:
                return State.MOVE_TO_LAUNCH;

            // Moving from start to the left or right based on color to the first line
            case MOVE_TO_LAUNCH:
                _generalCounter = 0;
                return State.LAUNCH;

            // Hit the first line, now moving down the line towards the wall
            case LAUNCH:
                return State.MOVE_TO_BALL;


            case MOVE_TO_BALL:
                return State.STOPPED;

            // Moving towards the center plate, knocking the ball off and stopping on the pad


            case STOPPED:
                return State.STOPPED;
        }
        Logger.getInstance().WriteMessage("No next state for state" + current);
        return current;
    }

    // This is all the logic for determing with method to execute based on the state
    private void DoState(State state)
    {
        switch(state)
        {
            // Starts here, robot is stopped
            case START:
                Logger.getInstance().WriteMessage("State Started");
                _currentState = GetNextState(State.START);
                break;

            // Moving from start to the left or right based on color to the first line
            case MOVE_TO_LAUNCH:
                MoveToFirstBeacon();
                break;

            // Hit the first line, now moving down the line towards the wall
            case LAUNCH:
                MovingDownBeaconLine();
                break;

            case MOVE_TO_BALL:
                PressingFirstBeacon();
                break;

            // Moving towards the center plate, knocking the ball off and stopping on the pad


            case STOPPED:
                _left.Stop();
                _left.SetEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                _right.Stop();
                _right.SetEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                Logger.getInstance().WriteMessage("State Stopped - complete");
                break;
        }
    }

    // This is a handy method to return a name for a state
    private String GetStateName(State state) {
        switch(state)
        {
            // Starts here, robot is stopped
            case START:
                return "Start";

            // Moving from start to the left or right based on color to the first line
            case MOVE_TO_LAUNCH:
                return "MOVE_TO_LAUNCH";

            // Hit the first line, now moving down the line towards the wall
            case LAUNCH:
                return "LAUNCH";

            case MOVE_TO_BALL:
                return "MOVE_TO_BALL";

            // Moving towards the center plate, knocking the ball off and stopping on the pad


            case STOPPED:
                return "STOPPED";
        }
        return "UNKNOWN STATE" + state;
    }

    private void MoveToFirstBeacon() {
        double distance = _ultrasonic.getUltrasonicLevel();
        _generalCounter2 = 0;

        Motor insideMotor = _color == AllianceColor.Left ? _left : _right;
        Motor otherMotor = _color == AllianceColor.Left ? _right : _left;
        double lightFront = _frontLightSensor.getLightDetected();
        double lightBack = _backLightSensor.getLightDetected();
        if (_generalCounter2 <1) {

            double ticks = GeneralHelpers.CalculateDistanceEncode(20);
            insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(9), ticks);
            otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(9), ticks);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (_generalCounter2 ==1) {

            double outTicks = GeneralHelpers.CalculateDistanceEncode(-5);
            double inTicks = GeneralHelpers.CalculateDistanceEncode(5);
            insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(9), inTicks);
            otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(9), outTicks);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(!_left.IsBusy() && !_right.IsBusy())
        {
            _generalCounter2 ++;

        }
        if (_generalCounter2 ==2)
        {
            _currentState = GetNextState(_currentState);
        }


    }

    public void MovingDownBeaconLine() {
        _generalCounter= 0;
        int count =0;
        double color_left_red = _leftColorSensor.red();
        double color_left_blue = _leftColorSensor.blue();
        double distance = _ultrasonic.getUltrasonicLevel();
        if (_generalCounter == 0)
        {
            flipper.LaunchBall();
            _generalCounter++;
        }
        if (_generalCounter==1 && count < 50)
        {
            _sweeper.SetSpeed(100);
            count++;
        }
        else if (count > 45 && _generalCounter ==1)
        {
            _generalCounter = 2;
        }
        if (_generalCounter == 2)
        {
            flipper.LaunchBall();
            _generalCounter++;
        }
        if (_generalCounter ==3)
        {
            _currentState = GetNextState(_currentState);
        }


    }

    private void PressingFirstBeacon()  {

        double distance = _ultrasonic.getUltrasonicLevel();
        _generalCounter2 = 0;

        Motor insideMotor = _color == AllianceColor.Left ? _left : _right;
        Motor otherMotor = _color == AllianceColor.Left ? _right : _left;
        double lightFront = _frontLightSensor.getLightDetected();
        double lightBack = _backLightSensor.getLightDetected();
        if (_generalCounter2 <1) {

            double ticks = GeneralHelpers.CalculateDistanceEncode(20);
            insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(9), ticks);
            otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(9), ticks);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (_generalCounter2 ==1) {

            double outTicks = GeneralHelpers.CalculateDistanceEncode(-5);
            double inTicks = GeneralHelpers.CalculateDistanceEncode(5);
            insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(9), inTicks);
            otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(9), outTicks);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        if(!_left.IsBusy() && !_right.IsBusy())
        {
            _generalCounter2 ++;

        }
        if (_generalCounter2 ==2)
        {
            _currentState = GetNextState(_currentState);
        }

    }



    //endregion

//region Setup Code for Motors, Gryo, Light Sensor, Color Sensor, Ultrasonic

    private void SetupColorSensor() {
        _leftColorSensor = hardwareMap.colorSensor.get(Constants.LEFT_COLOR_SENSOR);
//        _rightColorSensor = hardwareMap.colorSensor.get(Constants.RIGHT_COLOR_SENSOR);
    }


    private void SetupUltrasonic() {
        _ultrasonic = hardwareMap.ultrasonicSensor.get(Constants.DISTANCE);
        //_ultrasonicLeft = hardwareMap.ultrasonicSensor.get(Constants.DISTANCELEFT);

    }

    private void SetupLightSensor() {
        _frontLightSensor = hardwareMap.lightSensor.get(Constants.LIGHT_SENSOR);
        Logger.getInstance().WriteMessage("Light Sensor " + _frontLightSensor.getConnectionInfo());
        _frontLightSensor.enableLed(true);
        _backLightSensor = hardwareMap.lightSensor.get(Constants.LIGHT_SENSOR_BACK);
        Logger.getInstance().WriteMessage("BackSensor " + _backLightSensor.getConnectionInfo());
        _backLightSensor.enableLed(true);
    }

    private void SetupGyro() {
        _gyro = (HiTechnicNxtGyroSensor) hardwareMap.gyroSensor.get(Constants.GYRO);
        Logger.getInstance().WriteMessage(_gyro.status());
    }

    private void SetupMotors() {
        _left = new Motor(DcMotorSimple.Direction.REVERSE, hardwareMap.dcMotor.get(Constants.LEFT_DRIVE_MOTOR), true);
        _right = new Motor(DcMotorSimple.Direction.REVERSE, hardwareMap.dcMotor.get(Constants.RIGHT_DRIVE_MOTOR), true);
        _flipper = new Motor(DcMotorSimple.Direction.FORWARD, hardwareMap.dcMotor.get(Constants.FLIPPER_MOTOR),true);
        _sweeper = new Motor(DcMotorSimple.Direction.FORWARD, hardwareMap.dcMotor.get(Constants.SWEEPER_MOTOR),true);

        _right.SetEncoderDirection(DcMotorSimple.Direction.FORWARD);
        _left.SetEncoderDirection(DcMotorSimple.Direction.REVERSE);

        _left.Stop();
        _right.Stop();
    }

    private void SetupServos()
    {
        _pusherServer = hardwareMap.servo.get(Constants.PUSHER_SERVO);
    }

//endregion
}
