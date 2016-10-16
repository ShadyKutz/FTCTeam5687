package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by stephen on 16/10/16.
 */

@Autonomous(name = "Concept: NullOp", group = "Concept")
public class TestOpMode extends OpMode {

    private DcMotor _leftMotor;
    private DcMotor _rightMotor;

    @Override
    public void init() {
        _leftMotor   = hardwareMap.dcMotor.get("left_drive");
        _rightMotor  = hardwareMap.dcMotor.get("right_drive");

        _leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        _rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void loop() {
        _leftMotor.setPower(100.0);
        _rightMotor.setPower(100.0);
    }
}
