package com.team5687.opmodes;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;
import com.team5687.Constants;
import com.team5687.controllers.DriveController;
import com.team5687.helpers.Logger;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by stephen on 16/10/16.
 */

@Autonomous(name = "Test Drive Controller", group = "Test")
public class TestOpMode extends OpMode {
    DriveController _drive = new DriveController();
    @Override
    public void init() {
        telemetry.addLine("Init()");
        Logger.getInstance().SetTelemetry(telemetry);
        Logger.getInstance().WriteMessage("TestOpMode::Init()");
        _drive = new DriveController();
        _drive.Init(hardwareMap);
    }

    @Override
    public void loop() {
        _drive.Loop();
    }
}
