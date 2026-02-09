package frc.robot.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalOutput;

public class UltrasonicSensor {
  private final DigitalOutput trigger;
  private final DigitalInput echo;
  private long startTime;
  private long endTime;
  private double distanceCm;

  public UltrasonicSensor(int triggerPort, int echoPort) {
    trigger = new DigitalOutput(triggerPort);
    echo = new DigitalInput(echoPort);
  }

  public void measureDistance() {
    // Send a 10µs pulse to trigger the sensor
    trigger.pulse(10e-6); // 10 microseconds

    // Wait for the echo pulse to start (timeout after 100ms)
    long timeout = System.currentTimeMillis() + 100;
    while (!echo.get() && System.currentTimeMillis() < timeout) {}

    // Record the start time of the echo pulse
    startTime = System.nanoTime();

    // Wait for the echo pulse to end
    timeout = System.currentTimeMillis() + 100;
    while (echo.get() && System.currentTimeMillis() < timeout) {}

    // Record the end time of the echo pulse
    endTime = System.nanoTime();

    // Calculate pulse duration in microseconds
    double pulseDurationMicroseconds = (endTime - startTime) / 1000.0;

    // Convert to distance (cm) using speed of sound
    distanceCm = pulseDurationMicroseconds * 0.0343 / 2;
  }

  public double getDistanceCm() {
    return distanceCm;
  }
}