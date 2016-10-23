package com.team5687.primitives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.team5687.helpers.Logger;

public class Motor
{
    private DcMotorSimple.Direction _direction;
    private DcMotor _motor;

    public Motor(DcMotorSimple.Direction direction, DcMotor motor)
    {
        _motor = motor;
        _direction = direction;
        _motor.setDirection(direction);
    }


    public void Stop()
    {
        _motor.setPower(0);
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
