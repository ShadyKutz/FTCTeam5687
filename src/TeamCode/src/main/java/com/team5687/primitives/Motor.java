package com.team5687.primitives;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

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

    public void Move(double power)
    {
        if(power > 0 && power <= 100)
            _motor.setPower(power);
    }
}
