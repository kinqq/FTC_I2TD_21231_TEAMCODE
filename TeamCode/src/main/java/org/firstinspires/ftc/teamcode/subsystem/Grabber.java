package org.firstinspires.ftc.teamcode.subsystem;

import static org.firstinspires.ftc.teamcode.util.Constants.*;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class Grabber {
    public final ServoImplEx grabber, pitch, roll, pivot;
    private int elePos;

    public Grabber(HardwareMap hardwareMap) {
        grabber = (ServoImplEx) hardwareMap.get(Servo.class, "grabber");
        pitch = (ServoImplEx) hardwareMap.get(Servo.class, "pitch");
        roll = (ServoImplEx) hardwareMap.get(Servo.class, "roll");
        pivot = (ServoImplEx) hardwareMap.get(Servo.class, "pivot");

        grabber.setPwmRange(new PwmControl.PwmRange(600, 2400));
        pitch.setPwmRange(new PwmControl.PwmRange(600, 2400));
        roll.setPwmRange(new PwmControl.PwmRange(600, 2400));
        pivot.setPwmRange(new PwmControl.PwmRange(600, 2400));
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
                double posErrorTick = Math.abs(grabber.getPosition() - target); // tick = angle / max_angle

                // Tuned for axon servo with 6.0V
                //TODO: Use Rev SPM.
                double MAX_TRAVEL = Math.toRadians(355); // tick = angle / max_angle
                double SEC_PER_RAD = 0.11 / Math.toRadians(60);
                double TIME_FACTOR = 1.4;
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
                double SEC_PER_RAD = 0.110 / Math.toRadians(60);
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

    public class Pivot implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();
        private double expectedTime;
        private final double target;

        public Pivot(double target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                double posErrorTick = Math.abs(pivot.getPosition() - target);

                double MAX_TRAVEL = Math.toRadians(355);
                double SEC_PER_RAD = 0.110 / Math.toRadians(60); //TODO: Tune this
                double TIME_FACTOR = 1.0;
                expectedTime = posErrorTick * MAX_TRAVEL * SEC_PER_RAD * TIME_FACTOR;

                initialized = true;
            }

            pivot.setPosition(target);
            return timer.seconds() <= expectedTime;
        }
    }

    public Action grab() {
        return new Grab(GRABBER_CLOSE);
    }

    public Action release() {
        return new Grab(GRABBER_OPEN);
    }

    public Action sweep() {
        return new ParallelAction(
            new Pitch(0.8),
            new Pivot(0.74),
            roll(0)
        );
    }

    public Action pitchUp() {
        return new ParallelAction(
            new Pitch(PITCH_UP),
            new Pivot(0.8)
        );
    }

    public Action readySampleGrab() {
        return new ParallelAction(
                new Pitch(PITCH_SUBMERSIBLE),
                new Pivot(PIVOT_SUBMERSIBLE)
        );
    }

    public Action performSampleGrab() {
        return new SequentialAction(
                new ParallelAction(
                        new Pitch(PITCH_DOWN + 0.03 * elePos / 500),
                        new Pivot(PIVOT_DOWN + 0.03 * elePos / 500)
                ),
                new Grab(GRABBER_CLOSE),
                readySampleGrab()
        );
    }

    public Action basketReady() {
        return new ParallelAction(
                new Pitch(PITCH_BASKET_READY),
                new Pivot(PIVOT_BASKET_READY),
                roll(180)
        );
    }

    public Action basketDepositReady() {
        return new ParallelAction(
                new Pitch(PITCH_BASKET),
                new Pivot(PIVOT_BASKET)
        );
    }

    public Action basketDeposit() {
        return new SequentialAction(
                release(),
                new Pitch(PITCH_BASKET_READY),
                new Pivot(PIVOT_BASKET_READY)
        );
    }

    public Action readySpecimenGrab() {
        return new ParallelAction(
                new Pitch(0.8),
                new Pivot(0.9),
                new Grab(GRABBER_OPEN),
                roll(0)
        );
    }

    public Action readySpecimenClip() {
        return new ParallelAction(
                new Pitch(0.6),
                new Pivot(0.5),
                roll(180)
        );
    }

    public Action readySpecimenClipFront() {
        return new ParallelAction(
            new Pitch(PITCH_CLIP),
            new Pivot(PIVOT_CLIP),
            roll(180)
        );
    }

    public Action performSpecimenClip() {
        return new ParallelAction(
            new Pitch(0.55),
            new Pivot(0.3)
        );
    }

    public Action roll(double degree) {
        double MIN_DEGREE = -180.0;
        double MAX_DEGREE = 180.0;

        degree = Range.clip(degree, MIN_DEGREE, MAX_DEGREE);

        // Convert the adjusted degree to ticks
        double TICK = ROLL_TICK_ON_ZERO + ROLL_TICK_PER_DEG * degree;

        return new Roll(TICK);
    }

    public void updateElePos(int elePos) {
        this.elePos = elePos;
    }
}
