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


public class LiftController {

    private DcMotor _rightMotor; // here as place holders for the loop function.
    private DcMotor _leftMotor;


    private Gamepad _gamepad;

    public void Init(HardwareMap map, Gamepad gamepad) {
        _leftMotor = map.dcMotor.get(Constants.LEFT_LIFT_MOTOR);
        _rightMotor = map.dcMotor.get(Constants.RIGHT_LIFT_MOTOR);


        _gamepad = gamepad;
    }

    public void Loop() {
        //Logger.getInstance().WriteMessage("leftstick"  + _gamepad.left_stick_y);
        //Logger.getInstance().WriteMessage("rightstick'" +  _gamepad.right_stick_y);
        _rightMotor.setPower(-_gamepad.left_stick_y);
        _leftMotor.setPower(_gamepad.right_stick_y);


    }
    // this is what starts all the motors and servos.

}
