package com.team5687.controllers;

/**
 * Created by RedDragon on 10/25/2016.
 */

import com.qualcomm.robotcore.eventloop.EventLoop;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.team5687.Constants;
import com.team5687.helpers.Logger;


public class LiftController {

    private DcMotor _rightMotor; // here as place holders for the loop function.
    private DcMotor _leftMotor;
    private DcMotor _arm;


    private Gamepad _gamepad;

    public void Init(HardwareMap map, Gamepad gamepad) {
        _leftMotor = map.dcMotor.get(Constants.LEFT_LIFT_MOTOR);
        _rightMotor = map.dcMotor.get(Constants.RIGHT_LIFT_MOTOR);
        _leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        _arm = map.dcMotor.get(Constants.ARM_MOTOR);


        _gamepad = gamepad;
    }

    public void Loop() {
        //Logger.getInstance().WriteMessage("leftstick"  + _gamepad.left_stick_y);
        //Logger.getInstance().WriteMessage("rightstick'" +  _gamepad.right_stick_y);
        if (_gamepad.right_bumper) {
            _rightMotor.setPower(100);
        } else if (_gamepad.left_bumper) {
            _leftMotor.setPower(-100);
        } else if (_gamepad.left_trigger > 0) {
            _leftMotor.setPower(100);
        } else if (_gamepad.right_trigger > 0) {
            _rightMotor.setPower(-100);
        }
        else if (_gamepad.dpad_up) {
            _rightMotor.setPower(100);
            _leftMotor.setPower(-100);
        } else if (_gamepad.dpad_down) {
            _rightMotor.setPower(-100);
            _leftMotor.setPower(100);
        } else {
            _rightMotor.setPower(0);
            _leftMotor.setPower(0);
        }


        if (_gamepad.x) {
            _arm.setPower(.95);
        } else if (_gamepad.b) {
            _arm.setPower(-.45);
        }
        else if (_gamepad.a)
        {
            _arm.setPower(0);
        }
        else if (_gamepad.y)
        {
            _arm.setPower(.3);
        }


    }


    }

    // this is what starts all the motors and servos.


