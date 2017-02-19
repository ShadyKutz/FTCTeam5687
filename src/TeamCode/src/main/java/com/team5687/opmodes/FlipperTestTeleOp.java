package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team5687.controllers.DriveController;
import com.team5687.controllers.FlipperController;
import com.team5687.controllers.JoystickController;
import com.team5687.helpers.Logger;

@TeleOp(name = "Flipper Teleop", group = "Test")
public class FlipperTestTeleOp extends OpMode {
    JoystickController _drive = new JoystickController();
    FlipperController _flipper;
    @Override
    public void init() {
        telemetry.addLine("Init()");
        Logger.getInstance().SetTelemetry(telemetry);
        Logger.getInstance().WriteMessage("TeleOpTest::Init()");
        _drive = new JoystickController();
        _drive.Init(hardwareMap, gamepad1);
        _flipper = new FlipperController();
        _flipper.Init(hardwareMap);
    }

    @Override
    public void loop() {
        _drive.Loop();
        _flipper.Loop();
        if(gamepad1.a) {
            _flipper.LaunchBall();
        }
    }
}
