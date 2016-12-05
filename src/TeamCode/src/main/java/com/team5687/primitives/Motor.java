package com.team5687.primitives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.team5687.helpers.Logger;

import java.util.Set;

public class Motor
{
    private DcMotorSimple.Direction _direction;
    private DcMotorSimple.Direction _encoderDirection;

    private DcMotor _motor;
    private Boolean _useEncoders;

    public Motor(DcMotorSimple.Direction direction, DcMotor motor, Boolean useEncoders)
    {
        _encoderDirection = DcMotorSimple.Direction.FORWARD;
        _useEncoders = useEncoders;
        _motor = motor;
        _direction = direction;
        _motor.setDirection(direction);
        _motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        _motor.setMaxSpeed(1120);
        // Will check if we are using encoders, and adjust accordingly
        SetEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void SetEncoderDirection(DcMotorSimple.Direction direction) {
        _encoderDirection = direction;
    }

    public DcMotor Motor() {
        return _motor;
    }

    public void Stop()
    {
        SetEncoderMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SetEncoderMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motor.setPower(0);
        _motor.setTargetPosition(_motor.getCurrentPosition());
    }

    public int GetEncoderPosition() {
        return _motor.getCurrentPosition() < 0 ? _motor.getCurrentPosition() * -1 : _motor.getCurrentPosition();
    }

    public int GetTargetEncoderPosition() {
        return _motor.getTargetPosition() < 0 ? _motor.getTargetPosition() * -1 : _motor.getTargetPosition();
    }

    public void SetEncoderMode(DcMotor.RunMode mode) {
        if(_useEncoders)
            _motor.setMode(mode);
        else
            _motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    public void SetSpeed(double speed) {
        _motor.setPower(speed);
    }

    public void SetTargetEncoderPosition(double speed , double counts) {
        if(!_useEncoders)
            throw new RuntimeException("Motor not configured to use encoders");

        int value = _encoderDirection == DcMotorSimple.Direction.FORWARD ? (int)counts : (int)-counts;
        //int value = (int)counts;
        int targetCount = _motor.getCurrentPosition() + value;

        _motor.setPower(speed);
        _motor.setTargetPosition(targetCount);
        //SetEncoderMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public Boolean IsBusy () {
        if(!_useEncoders)
            return false;
        return _motor.isBusy();
    }

    public void MoveForward(double power)
    {
        _motor.setDirection(_direction);
        if(power > 0 && power <= 100)
            _motor.setPower(power);
        else
            Logger.getInstance().WriteMessage("Invalid Power passed to Motor::MoveForward()");
    }

    public void MoveBackward(double power)
    {
        if(_direction == DcMotorSimple.Direction.FORWARD)
            _motor.setDirection(DcMotorSimple.Direction.REVERSE);
        else
            _motor.setDirection(DcMotorSimple.Direction.FORWARD);

        if(power > 0 && power <= 100)
            _motor.setPower(power);
        else
            Logger.getInstance().WriteMessage("Invalid Power passed to Motor::MoveBackward()");

    }
}
