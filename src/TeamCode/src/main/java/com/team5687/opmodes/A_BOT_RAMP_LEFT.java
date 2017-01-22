package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team5687.Constants;
import com.team5687.helpers.GeneralHelpers;
import com.team5687.helpers.Logger;
import com.team5687.primitives.Motor;

import java.util.Timer;

/**
 * Created by stephen on 4/12/16.
 */

@Autonomous(name = "A-BOT-LEFT-RAMP", group = "real")
public class A_BOT_RAMP_LEFT extends LinearOpMode {


    Motor _left;
    Motor _right;
    int _done =0;
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        Motor _left;
        Motor _right;

        Logger.getInstance().SetTelemetry(telemetry);
        _left = new Motor(DcMotorSimple.Direction.FORWARD, hardwareMap.dcMotor.get(Constants.LEFT_DRIVE_MOTOR), true);
        _right = new Motor(DcMotorSimple.Direction.REVERSE, hardwareMap.dcMotor.get(Constants.RIGHT_DRIVE_MOTOR), true);


        _left.SetEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _right.SetEncoderMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        _left.Stop();
        _right.Stop();

        String message = String.format("Left Motor: %s Right Motor: %s", _left.IsBusy() ? "Busy" : "Not Busy", _right.IsBusy() ? "Busy" : "Not Busy");
        Logger.getInstance().WriteMessage(message);


        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        _right.SetSpeed(.5);
        _left.SetSpeed(.5);
        runtime.reset();
        while ((runtime.seconds() < 3.0)&& opModeIsActive()) {

        }

        // Step 2:  Spin right for 1.3 seconds
        _right.SetSpeed(-5);
        _left.SetSpeed(.5);
        runtime.reset();
        while ( (runtime.seconds() < 1.3)&& opModeIsActive()) {

        }

        // Step 3:  Drive Backwards for 1 Second
        _right.SetSpeed(-.5);
        _left.SetSpeed(-.5);
        runtime.reset();
        while ((runtime.seconds() < 1.0)&& opModeIsActive()) {

        }
        sleep(1000);


    }
}