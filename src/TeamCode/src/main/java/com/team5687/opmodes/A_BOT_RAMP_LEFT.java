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


    DcMotor _left;
    DcMotor _right;
    int _done =0;
    ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {


        Logger.getInstance().SetTelemetry(telemetry);
        _left = hardwareMap.dcMotor.get(Constants.LEFT_DRIVE_MOTOR);
        _right = hardwareMap.dcMotor.get(Constants.RIGHT_DRIVE_MOTOR);

        _right.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();


        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward for 3 seconds
        _right.setPower(.5);
        _left.setPower(.5);
        runtime.reset();
        while ((runtime.seconds() < 1.1)&& opModeIsActive()) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();

        }

        // Step 2:  Spin right for .5 seconds
        _right.setPower(.5);
        _left.setPower(-.5);
        runtime.reset();
        while ( (runtime.seconds() < 1.3)&& opModeIsActive()) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();

        }

        // Step 3:  Drive forward for 3 Second
        _right.setPower(.5);
        _left.setPower(.5);
        runtime.reset();
        while ((runtime.seconds() < 2.0)&& opModeIsActive()) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        _right.setPower(0);
        _left.setPower(0);
        runtime.reset();
        while ((runtime.seconds() < 3.0)&& opModeIsActive()) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(10000);


    }
}
