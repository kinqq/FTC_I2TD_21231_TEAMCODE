package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.*;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

public class Sequence {
    // Inner class to represent a command with its target, position, and delay
    private static class Command {
        Object target;
        double position;
        int delay;

        public Command(Object target, double position, int delay) {
            this.target = target;
            this.position = position;
            this.delay = delay;
        }

        public void execute() {
            // Add specific handling for different types of objects (e.g., Servo, DcMotor)
            if (target instanceof Servo) {
                ((Servo) target).setPosition(position);
            } else if (target instanceof DcMotorEx) {
                ((DcMotorEx) target).setTargetPosition((int) position);
                ((DcMotorEx) target).setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                ((DcMotorEx) target).setPower(1.0);
            }
        }
    }

    private final Map<String, List<Command>> sequences = new HashMap<>();
    private List<Command> currentSequence;
    private final ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();

    // Method to create a new sequence by name
    public Sequence create(String name) {
        currentSequence = new ArrayList<>();
        sequences.put(name, currentSequence);
        return this;
    }

    // Method to add a command to the current sequence
    public Sequence add(Object target, double position, int delay) {
        if (currentSequence != null) {
            currentSequence.add(new Command(target, position, delay));
        }
        return this;
    }

    // Method to finalize the current sequence
    public void build() {
        currentSequence = null;
    }

    // Method to run a sequence by name
    public void run(String name) {
        List<Command> sequence = sequences.get(name);
        if (sequence != null) {
            // Start executing the sequence
            executeSequence(sequence.iterator(), 0);
        } else {
            System.out.println("Sequence not found: " + name);
        }
    }

    // Helper method to execute commands sequentially
    private void executeSequence(Iterator<Command> iterator, int initialDelay) {
        if (iterator.hasNext()) {
            Command command = iterator.next();
            executor.schedule(() -> {
                command.execute();
                executeSequence(iterator, command.delay);
            }, initialDelay, TimeUnit.MILLISECONDS);
        }
    }

    // Method to shut down the executor when done
    public void shutdown() {
        executor.shutdown();
    }
}