package com.team5687.opmodes;

import android.graphics.Path;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team5687.controllers.GateController;
import com.team5687.controllers.JoystickController;

import com.team5687.controllers.SpinnerController;
import com.team5687.helpers.Logger;

/**
 * Created by RedDragon on 11/4/2016.
 */
@TeleOp(name = "TeleBot_Teleop", group = "Test")
public class TeleopBot_Teleop extends OpMode {
    JoystickController _Drive = new JoystickController();
    SpinnerController _spin = new SpinnerController();
    GateController _gate = new GateController();

    @Override
    public void init() {
        _Drive = new JoystickController();
        _Drive.Init(hardwareMap, gamepad1);
        _spin.Init(hardwareMap, gamepad1);
       _gate.Init(hardwareMap, gamepad1);
    }

    @Override
    public void loop() {
        _Drive.Loop();
        _spin.Loop();
        _gate.Loop();

    }
}
