package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.Elevator;
import org.firstinspires.ftc.teamcode.subsystem.Grabber;
import static org.firstinspires.ftc.teamcode.teleop.RobotMap.drive;
import static org.firstinspires.ftc.teamcode.util.Constants.*;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(name = "TestLLGrabber", group = "Test")
@Config
public class TestLLGrabber extends LinearOpMode {
    private Limelight3A limelight;
    private Grabber grabber;
    private Elevator elevator;
    private Drivetrain drive;

    public static final double X_FACTOR = -0.1;
    public static final double Y_FACTOR = 0.1;
    public static final double Y_OFFSET = 10;
    private static final int SMOOTHING_WINDOW_SIZE = 30; // Use last 30 readings for filtering

    private int elePos = 0;
    private int rotPos = 380;
    private double rotAngle = 29;
    private double eleExt = 0;
    private boolean isSampleFound = false;
    private double avgX, avgY, avgAngle;

    // Buffers for smoothing sensor values
    private final Queue<Double> xBuffer = new LinkedList<>();
    private final Queue<Double> yBuffer = new LinkedList<>();
    private final Queue<Double> angleBuffer = new LinkedList<>();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() {
        initializeHardware();
        displayReadyMessage();

        waitForStart();

        while (!isStopRequested()) {
            processGamepadInput();
            if (!isSampleFound) processVisionData();
            else performGrab();
            updateTelemetry();
        }
    }

    public void performGrab() {
        elevator.elevateTo(elePos - 150);
        elevator.rotateTo(ROT_DOWN);
        grabber.readySampleGrab().run(packet);
        grabber.roll(avgAngle);
        new SequentialAction(
                new SleepAction(1.5),
                grabber.performSampleGrab()
        ).run(packet);
    }

    private void initializeHardware() {
        grabber = new Grabber(hardwareMap);
        elevator = new Elevator(hardwareMap);
        drive = new Drivetrain(hardwareMap);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start(); // Start polling for data

        grabber.grabber.setPosition(GRABBER_OPEN);
        grabber.pitch.setPosition(0.36);
        grabber.pivot.setPosition(0.57);
    }

    private void displayReadyMessage() {
        telemetry.addData(">", "Robot Ready. Press Play.");
        telemetry.update();
    }

    private void processGamepadInput() {
        if (elevator.getElevatorPosition() < 1000) {
            elevator.elevatePower(0.2);
        }

        int eleAdjustment = (gamepad1.a ? 10 : 0) - (gamepad1.b ? 10 : 0);
        elePos += eleAdjustment;

        eleExt = 15.3 * elevator.getElevatorPosition() / 500;
        rotAngle = Math.toDegrees(Math.asin((40.132 * Math.sin(Math.toRadians(29))) / (40.132 + eleExt)));
        rotPos = (int) ((90 - rotAngle) * 580 / 83);
        grabber.pitch.setPosition(0.36);
        grabber.pivot.setPosition(0.57 - 0.03 * elePos / 800);

        elevator.rotateTo(rotPos);
    }

    private void processVisionData() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.getPythonOutput().length >= 4) {
            double[] pythonOutput = result.getPythonOutput();
            double valid = pythonOutput[0];
            double x = pythonOutput[1];
            double y = pythonOutput[2];
            double angle = -pythonOutput[3];

            if (valid == 1) {
                addToBuffer(xBuffer, x);
                addToBuffer(yBuffer, y);
                addToBuffer(angleBuffer, angle);
            }

            if (xBuffer.size() >= SMOOTHING_WINDOW_SIZE) {
                isSampleFound = true;
                avgX = getAverage(xBuffer);
                avgY = getAverage(yBuffer);
                avgAngle = getAverage(angleBuffer);
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }
    }

    private void updateTelemetry() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        telemetry.addData("rotAngle", rotAngle);
        telemetry.addData("rotPos", rotPos);
        telemetry.addData("rotEnc", elevator.getRotationPosition());
        telemetry.addData("elePos", elePos);
        telemetry.addData("eleEnc", elevator.getElevatorPosition());
        telemetry.addData("data collected", xBuffer.size());
        telemetry.update();
    }

    /**
     * Adds a new value to the buffer while maintaining a fixed window size.
     */
    private void addToBuffer(Queue<Double> buffer, double newValue) {
        if (buffer.size() >= SMOOTHING_WINDOW_SIZE) {
            buffer.poll(); // Remove oldest value
        }
        buffer.add(newValue);
    }

    /**
     * Computes the average of the values stored in the buffer.
     */
    private double getAverage(Queue<Double> buffer) {
        if (buffer.isEmpty()) return 0;
        double sum = 0;
        for (double val : buffer) {
            sum += val;
        }
        return sum / buffer.size();
    }
}
