package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.team5687.Constants;
import com.team5687.controllers.DriveController;
import com.team5687.helpers.Logger;

/**
 * Created by stephen on 23/10/16.
 */

@Autonomous(name = "Test Light Sensor", group = "Test")
public class LightTestOpMode extends OpMode {
    LightSensor lightSensor;  // Hardware Device Object
    @Override
    public void init() {
        Logger.getInstance().SetTelemetry(telemetry);
        lightSensor = hardwareMap.get(LightSensor.class, Constants.LIGHT_SENSOR);
    }

    @Override
    public void loop() {
        telemetry.addData("Raw", lightSensor.getRawLightDetected());
        telemetry.addData("Normal", lightSensor.getLightDetected());
    }
}
