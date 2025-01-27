package com.team5687.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.team5687.Constants;

public class SpinnerController {
    private Boolean _isActive = false;
    private Boolean isfoward = false;

    private DcMotor _motor;
    private Servo _gate;
    private Gamepad _gamepad;
    private Gamepad _gamepad2;

    public void Init(HardwareMap map, Gamepad gampad, Gamepad gampad2) {
        _motor = map.dcMotor.get(Constants.SWEEPER_MOTOR);
        _motor.setDirection(DcMotorSimple.Direction.FORWARD);
        _gate = map.servo.get(Constants.GATE_SERVO);

        _gamepad = gampad;
        _gamepad2 = gampad2;

    }

    public void Loop() {
        if (_gamepad.y || _gamepad2.dpad_left)
        {
            _gate.setPosition(0);
        }
        if (_gamepad.a || _gamepad2.dpad_right)
        {
            _gate.setPosition(.67);
        }
        if (_gamepad.left_bumper) {
            _isActive = true;
            isfoward = true;
        }
        else if (_gamepad.right_bumper)
        _isActive = false;

        if (_gamepad.dpad_down) {
            isfoward = false;
            _isActive = true;
        }
        else if (_gamepad.right_bumper)
        _isActive = false;


        if(_isActive && isfoward)
            _motor.setPower(.6);
        else if (_isActive && isfoward == false)
            _motor.setPower(-.6);
        else
            _motor.setPower(_gamepad2.left_stick_y);


    }
}
