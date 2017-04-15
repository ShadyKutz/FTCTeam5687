package com.team5687.controllers;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.team5687.Constants;
import com.team5687.helpers.Logger;

public class PusherController {
    private final double MID = 0.36;
    private final double LEFT = 0.67;
    private final double RIGHT = 0;

    private double _value = 0.0;
    private Boolean _isActive = false;
    private Servo _servo;
    private Gamepad _gamepad;

    public void Init(HardwareMap map, Gamepad gampad) {
        _servo = map.servo.get(Constants.GATE_SERVO);
        _gamepad = gampad;
    }

    public void Loop() {
        if(_gamepad.dpad_left)
            _value = LEFT;
        else if(_gamepad.dpad_right)
            _value = RIGHT;

        _servo.setPosition(_value);

        String message = String.format("Servo: %.2f", _value);
        Logger.getInstance().WriteMessage(message);
    }
}
