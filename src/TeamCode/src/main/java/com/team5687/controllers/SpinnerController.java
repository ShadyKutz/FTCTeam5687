package com.team5687.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.team5687.Constants;

public class SpinnerController {
    private Boolean _isActive = false;
    private Boolean isfoward = false;

    private DcMotor _motor;
    private Gamepad _gamepad;

    public void Init(HardwareMap map, Gamepad gampad) {
        _motor = map.dcMotor.get(Constants.SWEEPER_MOTOR);
        _motor.setDirection(DcMotorSimple.Direction.FORWARD);
        _gamepad = gampad;
    }

    public void Loop() {
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
            _motor.setPower(0);


    }
}
