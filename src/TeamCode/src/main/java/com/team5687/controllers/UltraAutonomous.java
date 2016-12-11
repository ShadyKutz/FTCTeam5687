package com.team5687.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.team5687.Constants;
import com.team5687.helpers.Logger;
import com.team5687.primitives.Motor;

import java.util.Random;

public class UltraAutonomous {
    public enum State {
        MovingForward,
        TurningLeft,
        TurningRight,
        Reversing
    }
    private Motor _leftMotor;
    private Motor _rightMotor;
    private UltrasonicSensor _distance;
    private UltrasonicSensor _distance2;
    private Boolean Turn;


    private State _state;
    private Random _random;

    private double _minDistance = 45.0;
    private double _min2Distance = 45.0;

    public void Init(HardwareMap map) {
        _leftMotor = new Motor(DcMotorSimple.Direction.REVERSE, map.dcMotor.get(Constants.LEFT_DRIVE_MOTOR), false);
        _rightMotor = new Motor(DcMotorSimple.Direction.FORWARD, map.dcMotor.get(Constants.RIGHT_DRIVE_MOTOR), false);
        _distance = map.get(UltrasonicSensor.class, Constants.BACK_RANGE_SENSOR);
        _distance2 =map.get(UltrasonicSensor.class, Constants.FRONT_RANGE_SENSOR);
        _state = State.MovingForward;


        _leftMotor.Stop();
        _rightMotor.Stop();
        _random = new Random();
    }

    public void Loop() {
        double distance = _distance.getUltrasonicLevel() - 2; // approx cm;
        double distance2 = _distance2.getUltrasonicLevel() - 2; // approx cm;
        _state = State.MovingForward;

        if(_state == State.MovingForward) {
            if(distance <= _minDistance) {
                _leftMotor.Stop();
                _rightMotor.Stop();
                Turn=true;
                _state = _state.TurningLeft;
            }
            else
            {
                _leftMotor.MoveForward(50);
                _rightMotor.MoveForward(50);
            }
        }
        else if(_state == State.TurningLeft) {
            if(distance2 <= _min2Distance * 2 && Turn== true) {
                _leftMotor.MoveBackward(45);
                _rightMotor.MoveForward(100);
            }
            else
                _leftMotor.Stop();
            _rightMotor.Stop();
        }


            else
            _leftMotor.Stop();
        _rightMotor.Stop();
        }


    }




