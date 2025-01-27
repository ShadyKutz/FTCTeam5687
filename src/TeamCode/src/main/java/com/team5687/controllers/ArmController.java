package com.team5687.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.team5687.Constants;
import com.team5687.helpers.Logger;

public class ArmController {

    private final double backDump = .95;
    private final double Holding = 0.53;
    private final double frontDump = 0.12;
    private final double scoopHold = 0.39;
    private final double neutralpush = .3;

    private double _valueLeft = 0.0;
    private double _valueRight = 0.0;
    private Boolean _isActive = false;
    private Servo _leftArm;
    private Servo _rightArm;
    private Gamepad _gamepad;

    public void Init(HardwareMap map, Gamepad gampad1) {
        _leftArm = map.servo.get(Constants.LEFT_ARM);
        _rightArm = map.servo.get(Constants.RIGHT_ARM);


        _gamepad = gampad1;
       _valueLeft = Holding;

    }

    public void Loop() {
        _valueRight = neutralpush;

        if (_gamepad.x) {
            _valueLeft = backDump;

        }
        else if(_gamepad.b)
            _valueLeft = frontDump;
        else if (_gamepad.y)
            _valueLeft = scoopHold;
        else if (_gamepad.a)
            _valueLeft = Holding;
        else if (_gamepad.dpad_up)
            _valueLeft = .78;
        if(_gamepad.left_bumper)
        {
            _valueRight = .6;
        }
        else if (_gamepad.right_bumper)
        {
            _valueRight = .1;
        }
        else
        {
            _valueRight = neutralpush;
        }



        _leftArm.setPosition(_valueLeft);
        _rightArm.setPosition(_valueRight);


    }
}
