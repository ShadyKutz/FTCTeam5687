package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.team5687.Constants;

/**
 * Created by stephen on 28/01/17.
 */
@Autonomous(name = "Sensor: Ultrasonic", group = "Sensor")
public class UltrasonicTestOpMode  extends LinearOpMode {

    private UltrasonicSensor _ultrasonic;
    private UltrasonicSensor _ultrasonicLeft;

    public void runOpMode() {
        _ultrasonic = hardwareMap.ultrasonicSensor.get(Constants.DISTANCE);
        _ultrasonicLeft = hardwareMap.ultrasonicSensor.get(Constants.DISTANCELEFT);

        while(true) {
            double distance = _ultrasonic.getUltrasonicLevel();
            double  distanceLeft = _ultrasonicLeft.getUltrasonicLevel();
            double difference = _ultrasonic.getUltrasonicLevel() - _ultrasonicLeft.getUltrasonicLevel();

            telemetry.addData("d", distance);
            telemetry.addData("d_l", distance);
            telemetry.addData("delta  ", difference);
            telemetry.update();
        }
    }
}
