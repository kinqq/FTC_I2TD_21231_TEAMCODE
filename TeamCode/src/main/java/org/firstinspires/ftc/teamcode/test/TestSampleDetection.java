package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "FTC EasyOpenCV Block Detection")
public class TestSampleDetection extends LinearOpMode {

    OpenCvCamera webcam;
    String selectedColor = "red"; // Red, Blue, or Yellow

    @Override
    public void runOpMode() {

        // Initialize the camera
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        // Set the pipeline for block detection
        BlockDetectionPipeline pipeline = new BlockDetectionPipeline();
        webcam.setPipeline(pipeline);

        // Open the camera device
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming when the camera is opened
                webcam.startStreaming(640, 480, OpenCvCameraRotation.SIDEWAYS_RIGHT);

                FtcDashboard dashboard = FtcDashboard.getInstance();
                dashboard.startCameraStream(webcam, 10);
            }

            @Override
            public void onError(int errorCode) {
                // Handle any errors during camera opening
                telemetry.addData("Error", "Camera could not be opened. Error code: " + errorCode);
                telemetry.update();
            }
        });

        telemetry.addData("Status", "Waiting for color selection...");
        telemetry.update();

        // Gamepad input for color selection during initialization
        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.b) {
                selectedColor = "red";
            } else if (gamepad1.x) {
                selectedColor = "blue";
            } else if (gamepad1.y) {
                selectedColor = "yellow";
            }

            telemetry.addData("Selected Color", selectedColor);
            telemetry.addData("Press 'A' for Red", "");
            telemetry.addData("Press 'B' for Blue", "");
            telemetry.addData("Press 'X' for Yellow", "");
            telemetry.update();
        }

        pipeline.setSelectedColor(selectedColor); // Pass selected color to the pipeline

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // OpMode loop
        while (opModeIsActive()) {
            // Keep streaming from the webcam
            if (gamepad1.b) {
                selectedColor = "red";
            } else if (gamepad1.x) {
                selectedColor = "blue";
            } else if (gamepad1.y) {
                selectedColor = "yellow";
            }

            pipeline.setSelectedColor(selectedColor);
            sleep(50); // Delay to allow proper frame processing
        }

        // Stop the camera when OpMode is stopped
        webcam.stopStreaming();
    }

    // Pipeline for detecting blocks of selected color
    class BlockDetectionPipeline extends OpenCvPipeline {

    private Mat hsv = new Mat();     // Declare Mat outside processFrame to avoid reallocation
    private Mat mask = new Mat();
    private Mat red1 = new Mat();    // For red color detection (two ranges)
    private Mat red2 = new Mat();

    private String selectedColor = "none"; // Color selected during initialization

    public void setSelectedColor(String color) {
        this.selectedColor = color;
    }

    @Override
    public Mat processFrame(Mat input) {
        // Convert the frame to HSV (use pre-declared Mat to avoid memory leak)
        Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lowerBound, upperBound;
        switch (selectedColor) {
            case "red":
                // Red has two HSV ranges due to hue wrapping around
                Core.inRange(hsv, new Scalar(0, 100, 100), new Scalar(10, 255, 255), red1);
                Core.inRange(hsv, new Scalar(170, 100, 100), new Scalar(180, 255, 255), red2);
                Core.addWeighted(red1, 1.0, red2, 1.0, 0.0, mask);
                break;
            case "blue":
                lowerBound = new Scalar(100, 150, 0);
                upperBound = new Scalar(140, 255, 255);
                Core.inRange(hsv, lowerBound, upperBound, mask);
                break;
            case "yellow":
                lowerBound = new Scalar(20, 100, 100);
                upperBound = new Scalar(30, 255, 255);
                Core.inRange(hsv, lowerBound, upperBound, mask);
                break;
            default:
                return input;  // Return unprocessed frame if no color selected
        }

        // Optional: Apply Gaussian blur to reduce noise
        Imgproc.GaussianBlur(mask, mask, new Size(5, 5), 0);

        // Find contours in the processed image
        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        Point imageCenter = new Point(input.cols() / 2.0, input.rows() / 2.0);
        double closestDistance = Double.MAX_VALUE;
        RotatedRect closestBlock = null;

        for (MatOfPoint contour : contours) {
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect rect = Imgproc.minAreaRect(contour2f);

            Point[] box = new Point[4];
            rect.points(box);

            // Draw all blocks with pink
            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, box[i], box[(i + 1) % 4], new Scalar(255, 0, 255), 2);  // Pink color
            }

            // Find the center of the block
            Point blockCenter = rect.center;

            // Calculate distance to the center of the frame
            double distanceToCenter = Math.sqrt(Math.pow(blockCenter.x - imageCenter.x, 2) + Math.pow(blockCenter.y - imageCenter.y, 2));

            if (distanceToCenter < closestDistance) {
                closestDistance = distanceToCenter;
                closestBlock = rect;
            }
        }

        // Highlight the closest block with green
        if (closestBlock != null) {
            Point[] closestBox = new Point[4];
            closestBlock.points(closestBox);

            for (int i = 0; i < 4; i++) {
                Imgproc.line(input, closestBox[i], closestBox[(i + 1) % 4], new Scalar(0, 255, 0), 2);  // Green color
            }

            // Print position and angle of the closest block
            telemetry.addData("Position of Closest Block", closestBlock.center);
            telemetry.addData("Angle of Rotation", closestBlock.angle);
            telemetry.update();
        }

        return input; // Return the processed frame to display it
    }
}
}
