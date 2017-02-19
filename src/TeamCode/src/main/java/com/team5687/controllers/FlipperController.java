package com.team5687.controllers;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team5687.helpers.Logger;

import static com.team5687.Constants.FLIPPER_MOTOR;
import static com.team5687.Constants.LAUNCHER_MOTOR_RIGHT;

/**
 * Created by stephen on 19/02/17.
 */

public class FlipperController {

    private enum State {
        Idle,
        Launching,
        ReversingLaunch,
        ResetingPosition,

    }

    private static final int LAUNCH_TIME_IN_MS = 1500;
    private static final int LAUNCH_POWER = 100;

    private static final int REVERSING_LAUNCH_TIME_IN_MS = 750;
    private static final int REVERSING_LAUNCH_POWER = 10;

    private static final int RESET_POSITION_TIME_IN_MS = 750;
    private static final int RESET_POSITION_POWER = 20;

    private DcMotor _motor;

    private ElapsedTime period  = new ElapsedTime();

    private State _state = State.Idle;
    private Boolean _needsLaunch = false;

    public void Init(HardwareMap map) {
        _motor = map.dcMotor.get(FLIPPER_MOTOR);
        _motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void Loop() {
        if(_needsLaunch && _state == State.Idle) {
            _state = State.Launching;
        }
        String message = String.format("%s - %.1f %s", GetStateName(), period.milliseconds(), _needsLaunch ? "true" : "false");
        Logger.getInstance().WriteMessage(message);
        switch(_state) {
            case Idle:
                // Do nothing
                StopMotor();
                break;
            case Launching:
            {
                if(period.milliseconds() > LAUNCH_TIME_IN_MS) {
                    StopMotor();
                    MoveToNextState();
                }
                else {
                    _motor.setDirection(DcMotorSimple.Direction.REVERSE);
                    _motor.setPower(LAUNCH_POWER);
                }
                break;
            }
            case ReversingLaunch:
            {
                if(period.milliseconds() > REVERSING_LAUNCH_TIME_IN_MS) {
                    StopMotor();
                    MoveToNextState();
                }
                else {
                    _motor.setDirection(DcMotorSimple.Direction.FORWARD);
                    _motor.setPower(REVERSING_LAUNCH_POWER);
                }
                break;
            }
            case ResetingPosition:
            {
                if(period.milliseconds() > RESET_POSITION_TIME_IN_MS) {
                    _needsLaunch = false;
                    StopMotor();
                    MoveToNextState();
                }
                else {
                    _motor.setDirection(DcMotorSimple.Direction.REVERSE);
                    _motor.setPower(RESET_POSITION_POWER);
                }
                break;
            }
        }
    }

    public void LaunchBall() {
        _needsLaunch = true;
    }

    private void StopMotor() {
        _motor.setPower(0);
    }

    private String GetStateName() {
        switch(_state) {
            case Idle:
                return "Idle";
            case Launching:
               return "Launching";
            case ReversingLaunch:
                return "Reversing";
            case ResetingPosition:
                return "Reseting";
        }
        return "Unknown";
    }
    private void MoveToNextState() {
        period.reset();
        switch(_state) {
            case Idle:
                _state = State.Idle;
                break;
            case Launching:
                _state = State.ReversingLaunch;
                break;
            case ReversingLaunch:
                _state = State.ResetingPosition;
                break;
            case ResetingPosition:
                _state = State.Idle;
                break;
        }
    }
}
