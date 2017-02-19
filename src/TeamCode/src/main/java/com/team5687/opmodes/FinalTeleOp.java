package com.team5687.opmodes;

/**
 * Created by stephen on 19/02/17.
 */

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team5687.controllers.FlipperController;
import com.team5687.controllers.JoystickController;
import com.team5687.controllers.LiftController;
import com.team5687.controllers.SpinnerController;
import com.team5687.helpers.Logger;

@TeleOp(name = "Steves Final Overpowered All Mid TeleOP Mode", group = "Test")
public class FinalTeleOp extends OpMode {
    JoystickController _drive = new JoystickController();
    FlipperController _flipper = new FlipperController();
    SpinnerController _spin = new SpinnerController();
    LiftController _lift = new LiftController();

    @Override
    public void init() {
        telemetry.addLine("Init()");
        Logger.getInstance().SetTelemetry(telemetry);
        Logger.getInstance().WriteMessage("TeleOpTest::Init()");
        _drive = new JoystickController();
        _drive.Init(hardwareMap, gamepad1);
        _spin.Init(hardwareMap, gamepad1);
        _lift.Init(hardwareMap, gamepad2);
        _flipper.Init(hardwareMap);
    }

    @Override
    public void loop() {
        _drive.Loop();
        _flipper.Loop();
        _spin.Loop();
        _lift.Loop();
        if(gamepad2.dpad_left || gamepad2.dpad_right) {
            _flipper.LaunchBall();
        }
    }
}
