package com.team5687.primitives;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import static android.content.Context.SENSOR_SERVICE;

public class CompassAndroid implements SensorEventListener {
    private SensorManager _sensorManager;


    public CompassAndroid(Context context) {
        _sensorManager = (SensorManager) context.getSystemService(SENSOR_SERVICE);
    }

    public void Init() {
        _sensorManager.registerListener(this, _sensorManager.getDefaultSensor(Sensor.TYPE_ORIENTATION), SensorManager.SENSOR_DELAY_UI);
    }

    public void Stop() {
        _sensorManager.unregisterListener(this);
    }

    public float GetAzimuth() {
        return 0.0f;
    }

    public float[] _values = new float[5];
    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        _values = event.values;
    }
}
