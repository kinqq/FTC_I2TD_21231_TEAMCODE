package org.firstinspires.ftc.teamcode.test;

import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.util.HSV;
import org.firstinspires.ftc.teamcode.util.LED;

@TeleOp(name = "Color Sensor Test", group = "Test")
@Config
public class TestColorSensor extends LinearOpMode {
    NormalizedColorSensor colorSensor;
    LED led;

    final float[] hsvValues = new float[3];
    float GAIN = 1;
    boolean isBragging = false;

    @Override
    public void runOpMode() {
        waitForStart();
        Gamepad prevGamepad1 = new Gamepad();

        led = new LED(hardwareMap.get(RevBlinkinLedDriver.class, "led"));

        while (opModeIsActive()) {
            if (gamepad1.back && !prevGamepad1.back) {
                switchLight();
            }

            if (gamepad1.start && !prevGamepad1.start) {
                isBragging ^= true;
            }

            GAIN += (float) (gamepad1.dpad_up ? 0.01 : (gamepad1.dpad_down ? -0.01 : 0));

            colorSensor.setGain(GAIN);
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            Color.colorToHSV(colors.toColor(), hsvValues);
            HSV hsv = new HSV(hsvValues[0], hsvValues[1], hsvValues[2]);

            if (isBragging) led.setRainbow();
            else {
                if (hsv.getColorCategory().equals("Red"))
                    led.setRed();
                if (hsv.getColorCategory().equals("Blue"))
                    led.setBlue();
                if (hsv.getColorCategory().equals("Yellow"))
                    led.setYellow();
                if (hsv.getColorCategory().equals("N/A"))
                    led.turnOff();
            }

            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hsv.getHue())
                    .addData("Saturation", "%.3f", hsv.getSaturation())
                    .addData("Value", "%.3f", hsv.getValue());
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            gamepad1.copy(prevGamepad1);
        }
    }

    public void switchLight() {
        SwitchableLight light = (SwitchableLight) colorSensor;
        light.enableLight(!light.isLightOn());
    }
}
