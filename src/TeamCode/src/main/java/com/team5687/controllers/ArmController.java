package com.team5687.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.team5687.Constants;
import com.team5687.helpers.Logger;

public class ArmController {

    private final double outLeft = 0.9;
    private final double outRight = 0.1;
    private final double inLeft = 0.9;
    private final double inRight = 0.1;

    private double _valueLeft = 0.0;
    private double _valueRight = 0.0;
    private Boolean _isActive = false;
    private Servo _leftArm;
    private Servo _rightArm;
    private Gamepad _gamepad;

    public void Init(HardwareMap map, Gamepad gampad) {
        _leftArm = map.servo.get(Constants.LEFT_ARM);
        _rightArm = map.servo.get(Constants.RIGHT_ARM);

        _gamepad = gampad;
    }

    public void Loop() {
        if (_gamepad.x) {
            _valueLeft = outLeft;
            _valueRight = outRight;
        }
        else if(_gamepad.b)
            _valueLeft = inLeft;
            _valueRight = inRight;



        _leftArm.setPosition(_valueLeft);
        _rightArm.setPosition(_valueRight);

        String message = String.format("LeftArm: %.2f", _valueLeft);
        String message2 = String.format("RightArm: %.2f", _valueRight);
        Logger.getInstance().WriteMessage(message);
        Logger.getInstance().WriteMessage(message2);
    }
}
