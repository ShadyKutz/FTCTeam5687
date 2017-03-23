

package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team5687.Constants;
import com.team5687.controllers.FlipperController;
import com.team5687.helpers.GeneralHelpers;
import com.team5687.helpers.Logger;
import com.team5687.primitives.Motor;
import com.team5687.controllers.SpinnerController;



@Autonomous(name = "BLUE_FLICK_AND_RAMP", group = "Real")

public class BLUE_FLICK_RAMP extends LinearOpMode {

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
    private ElapsedTime period  = new ElapsedTime();


    public void Init(HardwareMap map) {
        _left = map.dcMotor.get(Constants.LEFT_DRIVE_MOTOR);
        _right = map.dcMotor.get(Constants.RIGHT_DRIVE_MOTOR);
        _motor = map.dcMotor.get(Constants.FLIPPER_MOTOR);
        _motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        _sweeper = map.dcMotor.get(Constants.SWEEPER_MOTOR);
        _sweeper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        encoderDrive(DRIVE_TICKS,  11.5,  11.5, 5.0);

        flipper(1);
        sweep(6);
        flipper(1);// S1: Forward 47 Inches with 5 Sec timeout
        encoderDrive(TURN_TICKS,   -6, 6, 4.0);
        encoderDrive(DRIVE_TICKS, 15, 15, 4.0);


        sleep(1000);     // pause for each to move

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
    public void flipper(int launch) { // uses sleep commands and motor power to launch and reset the ball
        if (opModeIsActive()) {

            _motor.setPower(1);
            sleep(700);
            _motor.setPower(-.2);
            sleep(300);
            _motor.setPower(0);
            sleep(500);



        }
    }


    public void sweep(int time) // uses time to run the sweeper, in order to load a second ball into the chute
    {
        _sweeper.setPower(1);
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
