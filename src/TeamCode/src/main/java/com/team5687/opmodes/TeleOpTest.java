package com.team5687.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team5687.controllers.JoystickController;
import com.team5687.controllers.LiftController;
import com.team5687.controllers.PusherController;
import com.team5687.controllers.SpinnerController;
import com.team5687.helpers.Logger;

/**
 * Created by RedDragon on 10/25/2016.
 */



@TeleOp(name = "AutoBot_Teleop", group = "Test")
public class TeleOpTest extends OpMode {


    JoystickController _Drive = new JoystickController();

    LiftController _Lift   = new LiftController();


    @Override
    public void init() {
        telemetry.addLine("Init()");
        Logger.getInstance().SetTelemetry(telemetry);
        Logger.getInstance().WriteMessage("TeleOpTest::Init()");
        _Drive = new JoystickController();
        _Drive.Init(hardwareMap, gamepad1);
        _Lift.Init(hardwareMap, gamepad1);


    }

    @Override
    public void loop() {
        _Drive.Loop();
        _Lift.Loop();

    }
}
