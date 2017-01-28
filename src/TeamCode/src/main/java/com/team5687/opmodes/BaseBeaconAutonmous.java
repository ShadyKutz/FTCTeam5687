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
import com.team5687.Constants;
import com.team5687.helpers.GeneralHelpers;
import com.team5687.helpers.Logger;
import com.team5687.primitives.CompassAndroid;
import com.team5687.primitives.Motor;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptNullOp;
import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import static java.lang.Thread.sleep;

public class BaseBeaconAutonmous extends OpMode {
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
        MOVING_TO_CENTER_BALL,

        STOPPED
    }

    private int _generalCounter;
    private int _generalCounter2;
    //region Hardware Fields
    private HiTechnicNxtGyroSensor _gyro;
    private LightSensor _frontLightSensor;
    private LightSensor _backLightSensor;
    private ColorSensor _leftColorSensor;
    private ColorSensor _rightColorSensor;
    private UltrasonicSensor _ultrasonic;
    private UltrasonicSensor _ultrasonicLeft;
    private Motor _left;
    private Motor _right;
    private Servo _pusherServer;
    private CompassAndroid _compass;
    //endregion

    private State _currentState = State.START;

    private AllianceColor _color;
    private BeaconSide _side;

    protected BaseBeaconAutonmous(AllianceColor color, BeaconSide side) {
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

        switch(current)
        {
            // Starts here, robot is stopped
            case START:
                return State.MOVE_TO_FIRST_BEACON_LINE;

                    // Moving from start to the left or right based on color to the first line
            case MOVE_TO_FIRST_BEACON_LINE:
                _generalCounter = 0;
                return State.MOVING_DOWN_FIRST_BEACON_LINE;

                    // Hit the first line, now moving down the line towards the wall
            case MOVING_DOWN_FIRST_BEACON_LINE:
                return State.PRESSING_FIRST_BEACON;


            case PRESSING_FIRST_BEACON:
                return State.STOPPED;

                    // first beacon has been pressed, moving to the second line
            case MOVING_TO_SECOND_BEACON_LINE:
                _generalCounter = 0;
                return State.MOVING_DOWN_SECOND_BEACON_LINE;

                    // moving down the second line towards the wall
            case MOVING_DOWN_SECOND_BEACON_LINE:
                return State.PRESSING_SECOND_BEACON;

                    // pressing second beacon until it's our color
            case PRESSING_SECOND_BEACON:
                return State.MOVING_TO_CENTER_LINE;


                    // finished both beacons, moving back to center line (red / blue lines)
            case MOVING_TO_CENTER_LINE:
                return State.MOVING_TO_CENTER_BALL;

                    // Moving towards the center plate, knocking the ball off and stopping on the pad
            case MOVING_TO_CENTER_BALL:
                return State.STOPPED;

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
            case MOVE_TO_FIRST_BEACON_LINE:
                MoveToFirstBeacon();
                break;

            // Hit the first line, now moving down the line towards the wall
            case MOVING_DOWN_FIRST_BEACON_LINE:
                MovingDownBeaconLine();
                break;

            case PRESSING_FIRST_BEACON:
                PressingFirstBeacon();
                break;

            // first beacon has been pressed, moving to the second line
            case MOVING_TO_SECOND_BEACON_LINE:
                LogNotImplementedState(state);
                break;

            // moving down the second line towards the wall
            case MOVING_DOWN_SECOND_BEACON_LINE:
                LogNotImplementedState(state);
                break;

            // pressing second beacon until it's our color
            case PRESSING_SECOND_BEACON:
                LogNotImplementedState(state);
                break;


            // finished both beacons, moving back to center line (red / blue lines)
            case MOVING_TO_CENTER_LINE:
                LogNotImplementedState(state);
                break;

            // Moving towards the center plate, knocking the ball off and stopping on the pad
            case MOVING_TO_CENTER_BALL:
                LogNotImplementedState(state);
                break;

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
            case MOVE_TO_FIRST_BEACON_LINE:
                return "MOVE_TO_FIRST_BEACON_LINE";

            // Hit the first line, now moving down the line towards the wall
            case MOVING_DOWN_FIRST_BEACON_LINE:
                return "MOVING_DOWN_FIRST_BEACON_LINE";

            case PRESSING_FIRST_BEACON:
                return "PRESSING_FIRST_BEACON";

            // first beacon has been pressed, moving to the second line
            case MOVING_TO_SECOND_BEACON_LINE:
                return "MOVING_TO_SECOND_BEACON_LINE";

            // moving down the second line towards the wall
            case MOVING_DOWN_SECOND_BEACON_LINE:
                return "MOVING_DOWN_SECOND_BEACON_LINE";

            // pressing second beacon until it's our color
            case PRESSING_SECOND_BEACON:
                return "PRESSING_SECOND_BEACON";

            // finished both beacons, moving back to center line (red / blue lines)
            case MOVING_TO_CENTER_LINE:
                return "MOVING_TO_CENTER_LINE";

            // Moving towards the center plate, knocking the ball off and stopping on the pad
            case MOVING_TO_CENTER_BALL:
                return "MOVING_TO_CENTER_BALL";

            case STOPPED:
                return "STOPPED";
        }
        return "UNKNOWN STATE" + state;
    }

    private void MoveToFirstBeacon() {
        double distance = _ultrasonic.getUltrasonicLevel();

        Motor insideMotor = _color == AllianceColor.Left ? _left : _right;
        Motor otherMotor = _color == AllianceColor.Left ? _right : _left;
        double lightFront = _frontLightSensor.getLightDetected();
        double lightBack = _backLightSensor.getLightDetected();
        if(lightFront > Constants.LINE_DETECTION_MINIMUM) {
            //Logger.getInstance().WriteMessage("Found Line " + light);


            Logger.getInstance().WriteMessage(GetStateName(_currentState) + ",No Line Detected," + lightFront);
            double outTicks = GeneralHelpers.CalculateDistanceEncode(100);
            double inTicks = GeneralHelpers.CalculateDistanceEncode(-100);
            otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(0), outTicks);
            insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(0), inTicks);

            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            //insideMotor.SetEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            _currentState = GetNextState(_currentState); // Move to the next state
        }
        else {
            Logger.getInstance().WriteMessage("No Line " + lightFront + "Distance" + distance);
            if(!_left.IsBusy() && !_right.IsBusy())
            {
                double ticks = GeneralHelpers.CalculateDistanceEncode(90);
                insideMotor.SetTargetEncoderPosition((int)GeneralHelpers.CalculateDistanceEncode(12), ticks);
                otherMotor.SetTargetEncoderPosition((int)GeneralHelpers.CalculateDistanceEncode(12), ticks);
                insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
                otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
                _generalCounter2 = 0;
            }
        }

    }

    public void MovingDownBeaconLine() {
        // IF the color we want to press is on the left


        double lightFront = _frontLightSensor.getLightDetected();
        double lightBack = _backLightSensor.getLightDetected();
        double distance = _ultrasonic.getUltrasonicLevel();
        double distanceLeft = _ultrasonicLeft.getUltrasonicLevel();
        double difference = distance-distanceLeft;
        Motor insideMotor = _color == AllianceColor.Left ? _left : _right;
        Motor otherMotor = _color == AllianceColor.Left ? _right : _left;


        if (lightBack < Constants.LINE_DETECTION_MINIMUM && _generalCounter2 < 1) {
            // We want to slow down the inside motor here
            Logger.getInstance().WriteMessage(GetStateName(_currentState) + ",No Line Detected," + lightFront);
            double outTicks = GeneralHelpers.CalculateDistanceEncode(100);
            double inTicks = GeneralHelpers.CalculateDistanceEncode(100);
            otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(2.5), outTicks);
            insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(2.5), inTicks);

            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            //insideMotor.SetEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            // no line found, so slow down the inside motor till we find the line


        } else if (lightBack > Constants.LINE_DETECTION_MINIMUM && _generalCounter2 < 1) {
            // We want to slow down the inside motor here
            Logger.getInstance().WriteMessage(GetStateName(_currentState) + ",No Line Detected," + lightFront);
            double outTicks = GeneralHelpers.CalculateDistanceEncode(100);
            double inTicks = GeneralHelpers.CalculateDistanceEncode(-100);
            otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(0), outTicks);
            insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(0), inTicks);

            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            //insideMotor.SetEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            // no line found, so slow down the inside motor till we find the line
            _generalCounter2 = 1;

        } else if (_generalCounter2 == 1 && lightFront < Constants.LINE_DETECTION_MINIMUM) {
            // We want to slow down the inside motor here
            Logger.getInstance().WriteMessage(GetStateName(_currentState) + ",No Line Detected," + lightFront);
            double outTicks = GeneralHelpers.CalculateDistanceEncode(100);
            double inTicks = GeneralHelpers.CalculateDistanceEncode(-100);
            otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(2), outTicks);
            insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(3), inTicks);

            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            //insideMotor.SetEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);


        } else if (_generalCounter2 == 1 && lightFront < Constants.LINE_DETECTION_MINIMUM) { // no line found, so slow down the inside motor till we find the line

            _generalCounter2 = 2;
            Logger.getInstance().WriteMessage(GetStateName(_currentState) + ",No Line Detected," + lightFront);
            double outTicks = GeneralHelpers.CalculateDistanceEncode(100);
            double inTicks = GeneralHelpers.CalculateDistanceEncode(-100);
            otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(0), outTicks);
            insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(0), inTicks);

            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            //insideMotor.SetEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);


        }
        else if (_generalCounter2 == 2 && difference < .5)
        {
            Logger.getInstance().WriteMessage(GetStateName(_currentState) + ",No Line Detected," + lightFront);
            double outTicks = GeneralHelpers.CalculateDistanceEncode(100);
            double inTicks = GeneralHelpers.CalculateDistanceEncode(100);
            otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(0), outTicks);
            insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(2), inTicks);

            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            //insideMotor.SetEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (distance <10 || distanceLeft < 10)
            {
                _generalCounter2 = 3;
            }

        }
        else if (_generalCounter2 == 2 && difference > .5)
        {
            Logger.getInstance().WriteMessage(GetStateName(_currentState) + ",No Line Detected," + lightFront);
            double outTicks = GeneralHelpers.CalculateDistanceEncode(100);
            double inTicks = GeneralHelpers.CalculateDistanceEncode(-100);
            otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(2), outTicks);
            insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(0), inTicks);

            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            //insideMotor.SetEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (distance <10 || distanceLeft < 10)
            {
                _generalCounter2 = 3;
            }
        }


        else {
            Logger.getInstance().WriteMessage(GetStateName(_currentState) + ",Line Detected," + lightFront + "Distance" + distance);
            double ticks = GeneralHelpers.CalculateDistanceEncode(100);
            distance = _ultrasonic.getUltrasonicLevel();
            _generalCounter2 ++;
            if (distance <10) {

                if (_generalCounter > 1 ) {
                    _currentState = GetNextState(_currentState);
                }
                else
                {
                    _generalCounter++;
                }


            }
            otherMotor.SetTargetEncoderPosition((int)GeneralHelpers.CalculateDistanceEncode(2), ticks);
            insideMotor.SetTargetEncoderPosition((int)GeneralHelpers.CalculateDistanceEncode(2), ticks);

            otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
            insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
private void PressingFirstBeacon(){
    int counter =0;
    int counter2 = 0;
    int BlueMin = 0;
    int RedMin = 0;
    Motor insideMotor = _color == AllianceColor.Left ? _left : _right;
    Motor otherMotor = _color == AllianceColor.Left ? _right : _left;



    double outTicks = GeneralHelpers.CalculateDistanceEncode(100);
    double inTicks = GeneralHelpers.CalculateDistanceEncode(-100);
    otherMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(0), outTicks);
    insideMotor.SetTargetEncoderPosition((int) GeneralHelpers.CalculateDistanceEncode(0), inTicks);

    otherMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);
    //insideMotor.SetEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    insideMotor.SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);  Logger.getInstance().WriteMessage(GetStateName(_currentState) + " COLORS blue," + _rightColorSensor.blue() + "Colors red" +_rightColorSensor.red());
    if( _rightColorSensor.blue() > BlueMin && _leftColorSensor.red()> RedMin && counter <2)
   {
       _pusherServer.setPosition(Constants.PUSHER_SERVO_MAX);
       counter ++;
   }
   else if ( _rightColorSensor.blue() > BlueMin && _leftColorSensor.red()> RedMin && counter <2)
   {
       _pusherServer.setPosition(.3);
       counter --;
   }
    else if (_rightColorSensor.red() > RedMin && _leftColorSensor.blue()> BlueMin && counter2 < 2)
   {
       _pusherServer.setPosition(Constants.PUSHER_SERVO_MIN);
       counter2 ++;
   }
   else if (_rightColorSensor.red() > RedMin && _leftColorSensor.blue()> BlueMin && counter2 > 2)
   {
       _pusherServer.setPosition(.3);
       counter2 --;
   }
    else if (_rightColorSensor.red() > 1 && _leftColorSensor.red()> 2 || _rightColorSensor.blue() > 1 && _leftColorSensor.blue()> 2)
   {
       _currentState = GetNextState(_currentState);
   }

}


    //endregion

//region Setup Code for Motors, Gryo, Light Sensor, Color Sensor, Ultrasonic

    private void SetupColorSensor() {
        _leftColorSensor = hardwareMap.colorSensor.get(Constants.LEFT_COLOR_SENSOR);
        _rightColorSensor = hardwareMap.colorSensor.get(Constants.RIGHT_COLOR_SENSOR);

    }


    private void SetupUltrasonic() {
        _ultrasonic = hardwareMap.ultrasonicSensor.get(Constants.DISTANCE);
        _ultrasonicLeft = hardwareMap.ultrasonicSensor.get(Constants.DISTANCELEFT);

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

        _right.SetEncoderDirection(DcMotorSimple.Direction.FORWARD);
        _left.SetEncoderDirection(DcMotorSimple.Direction.REVERSE);

        _left.Stop();
        _right.Stop();
    }

    private void SetupServos()
    {
        _pusherServer = hardwareMap.servo.get(Constants.RIGHT_ARM);
    }

//endregion
}
