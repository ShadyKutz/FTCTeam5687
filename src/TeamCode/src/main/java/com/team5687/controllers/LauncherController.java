package com.team5687.controllers;

/**
 * Created by RedDragon on 10/25/2016.
 */

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.team5687.Constants;
import com.team5687.helpers.Logger;


public class LauncherController{

    private DcMotor _rightMotor; // here as place holders for the loop function.
    private DcMotor _leftMotor;
    private Servo LeftArm;
    private Servo RighArm;

    private Gamepad _gamepad;

    public void Init(HardwareMap map, Gamepad gamepad) {
        _leftMotor = map.dcMotor.get(Constants.LAUNCHER_MOTOR_LEFT);
        _rightMotor = map.dcMotor.get(Constants.LAUNCHER_MOTOR_RIGHT);


        _gamepad = gamepad;
    }

    public void Loop() {

        if (_gamepad.dpad_left)
        {
            _leftMotor.setPower(-100);
            _rightMotor.setPower(100);
        }
        if (_gamepad.dpad_right)
        {
            _leftMotor.setPower(0);
            _rightMotor.setPower(0);
        }


    }
    // this is what starts all the motors and servos.

}
