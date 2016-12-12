package com.team5687.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.team5687.Constants;
import com.team5687.helpers.Logger;

public class PusherController {
    private final double MID = 0.36;
    private final double LEFT = 0.6;
    private final double RIGHT = 0.1;

    private double _value = 0.0;
    private Boolean _isActive = false;
    private Servo _servo;
    private Gamepad _gamepad;

    public void Init(HardwareMap map, Gamepad gampad) {
        _servo = map.servo.get(Constants.PUSHER_SERVO);
        _gamepad = gampad;
    }

    public void Loop() {
        if(_gamepad.x)
            _value = LEFT;
        else if(_gamepad.b)
            _value = RIGHT;
        else
            _value = MID;


        _servo.setPosition(_value);

        String message = String.format("Servo: %.2f", _value);
        Logger.getInstance().WriteMessage(message);
    }
}
