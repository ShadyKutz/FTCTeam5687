package com.team5687.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.team5687.Constants;

public class SpinnerController {
    private Boolean _isActive = false;

    private DcMotor _motor;
    private Gamepad _gamepad;

    public void Init(HardwareMap map, Gamepad gampad) {
        _motor = map.dcMotor.get(Constants.SWEEPER_MOTOR);
        _motor.setDirection(DcMotorSimple.Direction.REVERSE);
        _gamepad = gampad;
    }

    public void Loop() {
        if(_gamepad.left_bumper)
            _isActive = true;
        else if(_gamepad.right_bumper)
            _isActive = false;

        if(_isActive)
            _motor.setPower(1);
        else
            _motor.setPower(0);
    }
}
