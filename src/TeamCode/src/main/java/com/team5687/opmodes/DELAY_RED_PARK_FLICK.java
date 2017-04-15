/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team5687.Constants;
import com.team5687.controllers.FlipperController;
import com.team5687.helpers.GeneralHelpers;
import com.team5687.helpers.Logger;
import com.team5687.primitives.Motor;
import com.team5687.controllers.SpinnerController;


/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: PushbotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forwards, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backwards for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous(name = "DELAY_RED_FLICK_AND_PARK", group = "Real")

public class DELAY_RED_PARK_FLICK extends LinearOpMode {

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double    DRIVE_TICKS             = .5;
    static final double     TURN_TICKS              = .3;
    private static final int LAUNCH_TIME_IN_MS = 1500;
    private static final int LAUNCH_POWER = 100;
    private static final int REVERSING_LAUNCH_TIME_IN_MS = 750;
    private static final int REVERSING_LAUNCH_POWER = 10;
    private static final int RESET_POSITION_TIME_IN_MS = 750;
    private static final int RESET_POSITION_POWER = 20;
    FlipperController _flipper = new FlipperController();
    SpinnerController sweep = new SpinnerController();
    /* Declare OpMode members. */
    // Use a Pushbot's hardware
    private ElapsedTime     runtime = new ElapsedTime();
    private DcMotor _left;
    private DcMotor _right;
    private DcMotor _sweeper;
    private DcMotor FLIPPER_MOTOR;
    private DcMotor _motor;
    private Servo _arm;
    private ElapsedTime period  = new ElapsedTime();


    public void Init(HardwareMap map) {
        _left = map.dcMotor.get(Constants.LEFT_DRIVE_MOTOR);
        _arm = map.servo.get(Constants.GATE_SERVO);
        _right = map.dcMotor.get(Constants.RIGHT_DRIVE_MOTOR);
        _motor = map.dcMotor.get(Constants.FLIPPER_MOTOR);
        _motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _sweeper = map.dcMotor.get(Constants.SWEEPER_MOTOR);
        _sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _sweeper.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void runOpMode() {


        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */

        Init(hardwareMap);


        _right.setDirection(DcMotorSimple.Direction.REVERSE);
        _left.setDirection(DcMotorSimple.Direction.FORWARD);



        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();


        idle();

        _left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                _left.getCurrentPosition(),
                _right.getCurrentPosition());
        telemetry.update();



        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        sleep(10000);
        _arm.setPosition(0);
        encoderDrive(DRIVE_TICKS,  13,  13, 5.0);
        encoderDrive(TURN_TICKS, -5.3,5.3,5.0);

        flipper(1);
        _arm.setPosition(0);
        sweep(8);

        flipper(1);
        // S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_TICKS,   -5.4, 5.4, 5.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
        encoderDrive(DRIVE_TICKS, -12, -12, 4.0);  // S3: Reverse 24 Inches with 4 Sec timeout


        sleep(1000);     // pause for servos to move

        telemetry.addData("Path", "Complete");
        telemetry.update();
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void flipper(int launch) {
        if (opModeIsActive()) {

            _motor.setPower(1);
            sleep(700);
            _motor.setPower(-.2);
            sleep(300);
            _motor.setPower(0);
            sleep(500);



        }
    }


    public void sweep(int time)
    {
        _sweeper.setPower(.8);
        runtime.reset();

        while (opModeIsActive() &&
                (runtime.seconds() < time)) {

            telemetry.addData("Path2", "Running at %7d :%7d", 4, 4);
            telemetry.update();
        }
        _sweeper.setPower(0);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = _left.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = _right.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            _left.setTargetPosition(newLeftTarget);
            _right.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            _left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            _right.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            _left.setPower(Math.abs(speed));
            _right.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (_left.isBusy() && _right.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        _left.getCurrentPosition(),
                        _right.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            _left.setPower(0);
            _right.setPower(0);

            // Turn off RUN_TO_POSITION
            _left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            _right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



}
