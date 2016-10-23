package com.team5687.helpers;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by stephen on 23/10/16.
 */
public class Logger {
    private static Logger ourInstance = new Logger();

    public static Logger getInstance() {
        return ourInstance;
    }

    private Logger() {
    }

    private Telemetry _telemetry;

    public void SetTelemetry(Telemetry tele) {
        _telemetry = tele;
    }

    public void WriteMessage(String message) {
        _telemetry.addLine(message);
        _telemetry.update();
    }
}
