package org.firstinspires.ftc.teamcode.Utils;

import android.app.Notification;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Queue;

public interface IDevice {
    void readControls(GamePadReadings oldReadings, GamePadReadings newReadings);
    // void readMessageQueue();
    // void readSensors();
    void run(boolean autoMode);
}
