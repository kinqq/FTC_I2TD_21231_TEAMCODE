package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

public class LED {
    // Instance of the RevBlinkinLedDriver
    RevBlinkinLedDriver led;

    // Predefined color patterns for the LED
    RevBlinkinLedDriver.BlinkinPattern rainbow = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
    RevBlinkinLedDriver.BlinkinPattern blue = RevBlinkinLedDriver.BlinkinPattern.BLUE;
    RevBlinkinLedDriver.BlinkinPattern red = RevBlinkinLedDriver.BlinkinPattern.RED;
    RevBlinkinLedDriver.BlinkinPattern yellow = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
    RevBlinkinLedDriver.BlinkinPattern off = RevBlinkinLedDriver.BlinkinPattern.BLACK; // Off pattern (black)

    // Constructor to initialize the LED driver
    public LED(RevBlinkinLedDriver _led) {
        led = _led;
    }

    // Method to set the LED to rainbow pattern
    public void setRainbow() {
        led.setPattern(rainbow);
    }

    // Method to set the LED to blue pattern
    public void setBlue() {
        led.setPattern(blue);
    }

    // Method to set the LED to red pattern
    public void setRed() {
        led.setPattern(red);
    }

    // Method to set the LED to yellow pattern
    public void setYellow() {
        led.setPattern(yellow);
    }

    // Method to turn the LED off (black pattern)
    public void turnOff() {
        led.setPattern(off);
    }
}
