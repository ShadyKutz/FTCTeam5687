package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team5687.controllers.JoystickController;
import com.team5687.controllers.PusherController;
import com.team5687.controllers.SpinnerController;
import com.team5687.helpers.Logger;

/**
 * Created by RedDragon on 10/25/2016.
 */



@TeleOp(name = "TeleOptest", group = "Test")
public class TeleOpTest extends OpMode {

    public float Power1 =100;
    JoystickController _joy = new JoystickController();
    SpinnerController _spin = new SpinnerController();
    PusherController _pusher = new PusherController();


    @Override
    public void init() {
        telemetry.addLine("Init()");
        Logger.getInstance().SetTelemetry(telemetry);
        Logger.getInstance().WriteMessage("TeleOpTest::Init()");
        _joy = new JoystickController();
        _joy.Init(hardwareMap, gamepad1);
        _spin.Init(hardwareMap, gamepad1);
        _pusher.Init(hardwareMap, gamepad1);

        Logger.getInstance().WriteMessage(gamepad1 != null ? "Not Null" : "null");
    }

    @Override
    public void loop() {
        _joy.Loop();
        _spin.Loop();
        _pusher.Loop();
    }
}
