package com.team5687.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.team5687.Constants;
import com.team5687.helpers.Logger;
import com.team5687.primitives.Motor;

import java.util.Random;

public class DriveController {
    public enum State {
        MovingForward,
        TurningLeft,
        TurningRight,
        Reversing
    }
    private Motor _leftMotor;
    private Motor _rightMotor;
    private UltrasonicSensor _distance;

    private State _state;
    private Random _random;

    private double _minDistance = 45.0;

    public void Init(HardwareMap map) {
        _leftMotor = new Motor(DcMotorSimple.Direction.REVERSE, map.dcMotor.get(Constants.LEFT_DRIVE_MOTOR));
        _rightMotor = new Motor(DcMotorSimple.Direction.FORWARD, map.dcMotor.get(Constants.RIGHT_DRIVE_MOTOR));
        _distance = map.get(UltrasonicSensor.class, Constants.FRONT_RANGE_SENSOR);
        _state = State.MovingForward;


        _leftMotor.Stop();
        _rightMotor.Stop();
        _random = new Random();
    }

    public void Loop() {
        double distance = _distance.getUltrasonicLevel() - 2; // approx cm;

        if(_state == State.MovingForward) {
            if(distance <= _minDistance) {
                if(_random.nextBoolean())
                    _state = State.TurningLeft;
                else
                    _state = State.TurningRight;
            }
            else
            {
                _leftMotor.MoveForward(100);
                _rightMotor.MoveForward(100);
            }
        }
        else if(_state == State.TurningLeft) {
            if(distance < _minDistance * 2) {
                _leftMotor.MoveBackward(45);
                _rightMotor.MoveForward(100);
            }
            else
                _state = State.MovingForward;
        }
        else if(_state == State.TurningRight) {
            if(distance < _minDistance * 2) {
                _leftMotor.MoveForward(100);
                _rightMotor.MoveBackward(45);
            }
            else
                _state = State.MovingForward;
        }
        else if(_state == State.Reversing) {
            if(distance < 100.0) {
                _leftMotor.MoveBackward(100);
                _rightMotor.MoveBackward(100);
            }
            else
                _state = State.MovingForward;
        }

        String message = String.format("Drive Controller : State %s, Distance: %.2f", _state, distance);
        Logger.getInstance().WriteMessage(message);

    }


}
