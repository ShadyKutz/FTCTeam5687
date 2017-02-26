package com.team5687.opmodes;

        import android.graphics.Path;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.team5687.controllers.GateController;
        import com.team5687.controllers.JoystickController;
        import com.team5687.controllers.ArmController;
        import com.team5687.controllers.LauncherController;
        import com.team5687.controllers.SpinnerController;
        import com.team5687.helpers.Logger;

/**
 * Created by RedDragon on 11/4/2016.
 */
@TeleOp(name = "TeleBot_Teleop", group = "Test")
public class TeleopBot_Teleop extends OpMode {
    ArmController _Arm = new ArmController();
    JoystickController _Drive = new JoystickController();
    SpinnerController _spin = new SpinnerController();
    GateController _gate = new GateController();
    LauncherController _launch = new LauncherController();

    @Override
    public void init() {
        _Arm = new ArmController();
        _Drive = new JoystickController();
        _spin = new SpinnerController();
        _launch = new LauncherController();
        _launch.Init(hardwareMap,gamepad2);
        _Arm.Init(hardwareMap, gamepad2);
        _Drive.Init(hardwareMap, gamepad1);
        _spin.Init(hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        _launch.Loop();
        _Arm.Loop();
        _Drive.Loop();
        _spin.Loop();

    }
}
