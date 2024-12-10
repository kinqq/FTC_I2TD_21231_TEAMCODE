package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Grabber {
    private final ServoImplEx grabber, pitch, roll;



    public Grabber(HardwareMap hardwareMap) {
        grabber = (ServoImplEx) hardwareMap.get(Servo.class, "grabber");
        pitch = (ServoImplEx) hardwareMap.get(Servo.class, "pitch");
        roll = (ServoImplEx) hardwareMap.get(Servo.class, "roll");

        grabber.setPwmRange(new PwmControl.PwmRange(500, 2500));
        pitch.setPwmRange(new PwmControl.PwmRange(500, 2500));
        roll.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    public class Grab implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();
        private double expectedTime;
        private final double target;

        public Grab(double _target) {
            target = _target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                double posErrorTick = Math.abs(grabber.getPosition() - target);

                // Tuned for speed servo with 4.8V
                double MAX_TRAVEL = Math.toRadians(300);
                double SEC_PER_RAD = 0.11 / Math.toRadians(60);
                double TIME_FACTOR = 1.0;
                expectedTime = posErrorTick * MAX_TRAVEL * SEC_PER_RAD * TIME_FACTOR;

                initialized = true;
            }

            grabber.setPosition(target);
            return timer.seconds() <= expectedTime;
        }
    }

    public class Pitch implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();
        private double expectedTime;
        private final double target;

        public Pitch(double _target) {
            target = _target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                double posErrorTick = Math.abs(pitch.getPosition() - target);

                double MAX_TRAVEL = Math.toRadians(300);
                double SEC_PER_RAD = 0.25 / Math.toRadians(60);
                double TIME_FACTOR = 1.0;
                expectedTime = posErrorTick * MAX_TRAVEL * SEC_PER_RAD * TIME_FACTOR;

                initialized = true;
            }

            pitch.setPosition(target);
            return timer.seconds() <= expectedTime;
        }
    }

    public class Roll implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();
        private double expectedTime;
        private final double target;

        public Roll(double target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                double posErrorTick = Math.abs(roll.getPosition() - target);

                double MAX_TRAVEL = Math.toRadians(300);
                double SEC_PER_RAD = 0.055 / Math.toRadians(60);
                double TIME_FACTOR = 1.0;
                expectedTime = posErrorTick * MAX_TRAVEL * SEC_PER_RAD * TIME_FACTOR;

                initialized = true;
            }

            roll.setPosition(target);
            return timer.seconds() <= expectedTime;
        }
    }

    public Action grab() {
        return new Grab(GRABBER_CLOSE);
    }

    public Action release() {
        return new Grab(GRABBER_OPEN);
    }

    public Action pitchForward() {
        return new Pitch(PITCH_FORWARD);
    }

    public Action pitchUp() {
        return new Pitch(PITCH_UP);
    }

    public Action pitchBackward() {
        return new Pitch(PITCH_BACKWARD);
    }

    public Action pitchClip() {
        return new Pitch(PITCH_CLIP);
    }

    public Action pitchGrab() {
        return new Pitch(PITCH_GRAB);
    }

    public Action roll(double degree) {
        double MIN_DEGREE = -90.0;
        double MAX_DEGREE = 180.0;

        degree = Range.clip(degree, MIN_DEGREE, MAX_DEGREE);


        // Convert the adjusted degree to ticks
        double TICK = ROLL_TICK_ON_ZERO + ROLL_TICK_PER_DEG * degree;

        return new Roll(TICK);
    }

    public Action rollTo(double tick) {
        return new Roll(tick);
    }
}
