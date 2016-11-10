package com.team5687.controllers;

/**
 * Created by RedDragon on 11/4/2016.
 */
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.team5687.Constants;
import com.team5687.helpers.Logger;


public class GateController {
    private final double close = 0.3;
    private final double open = 0.6;
    private double _value = 0.0;
    private Boolean _isActive = false;
    private Servo _servo;
    private Gamepad _gamepad;

    public void Init(HardwareMap map, Gamepad gampad) {
        _servo = map.servo.get(Constants.GATE_SERVO);
        _gamepad = gampad;
    }

    public void Loop() {
        if(_gamepad.dpad_up)
            _value =open;
        else if(_gamepad.b)
            _value = close;

        _servo.setPosition(_value);

        String message = String.format("Servo: %.2f", _value);
        Logger.getInstance().WriteMessage(message);
    }

}
