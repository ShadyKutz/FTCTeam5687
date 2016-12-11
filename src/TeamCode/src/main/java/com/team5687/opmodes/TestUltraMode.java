package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.team5687.controllers.UltraAutonomous;
import com.team5687.helpers.Logger;

/**
 * Created by CHS-Student on 12/11/2016.
 */


    @Autonomous(name = "Ultrasonic", group = "Test")
    public class TestUltraMode extends OpMode {
        UltraAutonomous _drive = new UltraAutonomous();
        @Override
        public void init() {
            telemetry.addLine("Init()");
            Logger.getInstance().SetTelemetry(telemetry);
            Logger.getInstance().WriteMessage("TestOpMode::Init()");
            _drive = new UltraAutonomous();
            _drive.Init(hardwareMap);
        }

        @Override
        public void loop() {
            _drive.Loop();
        }
    }


