package org.firstinspires.ftc.teamcode.util;

public class HSV {
    private double hue;
    private double saturation;
    private double value;

    // Constructor to initialize HSV values
    public HSV(double hue, double saturation, double value) {
        if (hue < 0 || hue > 360) {
            throw new IllegalArgumentException("Hue must be between 0 and 360");
        }
        if (saturation < 0.0 || saturation > 1.0) {
            throw new IllegalArgumentException("Saturation must be between 0.0 and 1.0");
        }
        if (value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException("Value must be between 0.0 and 1.0");
        }
        this.hue = hue;
        this.saturation = saturation;
        this.value = value;
    }

    // Accessor methods
    public double getHue() {
        return hue;
    }

    public double getSaturation() {
        return saturation;
    }

    public double getValue() {
        return value;
    }

    // Method to determine if the color is red, yellow, or blue
    public String getColorCategory() {
        if (isInRange(hue, 20, 20))
            return "Red";
        else if (isInRange(hue, 70, 20))
            return "Yellow";
        else if (isInRange(hue, 220, 20))
            return "Blue";
        else
            return "N/A";
    }

    // Helper method to check if hue is within a certain range
    private boolean isInRange(double hue, int targetHue, int range) {
        return hue >= (targetHue - range) && hue <= (targetHue + range);
    }
}
