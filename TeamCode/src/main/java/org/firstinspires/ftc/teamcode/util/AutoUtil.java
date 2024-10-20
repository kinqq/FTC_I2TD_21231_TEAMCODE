package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class AutoUtil {
    LinearOpMode opMode;
    static ElapsedTime runtime = new ElapsedTime();

    // Declare motor variables
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;
    DcMotor eleLeft, eleRight, rotLeft, rotRight;
    Servo grabber;
    LED led;

    // Odometry driver
    GoBildaPinpointDriver odo;

    // Position
    double x = 0; //mm
    double y = 0; // mm
    double heading = 0; // Radian

    // Ticks per one revolution of the motor
    final double TICKS_PER_REV = 537.7;

    // Circumference of the wheel in mm
    final double WHEEL_CIRCUMFERENCE_MM = 96.0 * Math.PI;

    public AutoUtil(LinearOpMode linearOpMode) {
        this.opMode = linearOpMode;

        this.frontLeft = linearOpMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        this.frontRight = linearOpMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        this.backLeft = linearOpMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        this.backRight = linearOpMode.hardwareMap.get(DcMotorEx.class, "rightBack");

        this.eleLeft =  linearOpMode.hardwareMap.get(DcMotorEx.class, "leftEle");
        this.eleRight =  linearOpMode.hardwareMap.get(DcMotorEx.class, "rightEle");
        this.rotLeft =  linearOpMode.hardwareMap.get(DcMotorEx.class, "leftRot");
        this.rotRight =  linearOpMode.hardwareMap.get(DcMotorEx.class, "rightRot");

        this.grabber = linearOpMode.hardwareMap.get(Servo.class, "grabber");

        eleLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        eleLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eleRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.odo = linearOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        this.led = new LED(linearOpMode.hardwareMap.get(RevBlinkinLedDriver.class, "led"));
        led.turnOff();

        this.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        this.backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    /**
     *
     * @param position This is for the elevator position, use stuff like ELE_HIGH not just numbers
     * @param power This is how fast the elevator will move
     * @param timeout The maximum that the elevator will move, I don't think this will be a problem but just in case
     */
    public void elevator(int position, double power, double timeout){
        eleLeft.setTargetPosition(position);
        eleLeft.setPower(power);
        eleLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        eleRight.setTargetPosition(position);
        eleRight.setPower(power);
        eleRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        runtime.reset();
        while ((opMode.opModeIsActive()) && runtime.seconds() < timeout) {
            int eleLeftDiff = Math.abs(eleLeft.getTargetPosition() - eleLeft.getCurrentPosition());
            int eleRightDiff = Math.abs(eleRight.getTargetPosition() - eleRight.getCurrentPosition());

            if (eleLeftDiff < 3 || eleRightDiff < 3) {
                break;
            }
            opMode.telemetry.addData("leftDiff", eleLeftDiff);
            opMode.telemetry.addData("rightDiff", eleRightDiff);
            opMode.telemetry.update();
        }

        eleLeft.setPower(0);
        eleRight.setPower(0);
    }

    /**
     *
     * @param position This is for the rotater position, use stuff like ROT_UP not just numbers
     * @param power This is how fast the rotation will move
     * @param timeout The maximum that it will rotate, I don't think this will be a problem but just in case
     */
    public void rotate(int position, double power, double timeout){
        runtime.reset();
        while ((opMode.opModeIsActive()) && runtime.seconds() < timeout) {
            rotLeft.setTargetPosition(position);
            rotLeft.setPower(power);
            rotLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotRight.setTargetPosition(position);
            rotRight.setPower(power);
            rotRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

    /**
     *
     * @param position Set to the position given, use GRABBER_OPEN and GRABBER_CLOSE
     */
    public void grabber(double position){
        grabber.setPosition(position);
    }


    // For some reason, distance is not really accurate
    /**
     * Strafes the robot in a specified direction for a given distance.
     * For some reason, distance is not really accurate. Enter 4/3 of actual distance you want to travel
     * TODO: this needs to be fixed
     *
     * @param distance The distance in millimeters the robot should strafe.
     * @param angle The angle in degrees at which to strafe relative to the robot's front.
     *              0 degrees means strafing directly to the right, 90 degrees means forward.
     * @param power The speed at which the robot should move (range 0.0 to 1.0).
     * @param timeout The maximum time in seconds to complete the strafe.
     */
    public void strafe(double distance, double angle, double power, double timeout) {
        // Convert distance in mm to target encoder counts
        final double TICKS_PER_MM = TICKS_PER_REV / WHEEL_CIRCUMFERENCE_MM;
        int targetPosition = (int) (distance * TICKS_PER_MM);

        // Convert angle to radians for calculations
        double robotAngle = Math.toRadians(angle);

        // Calculate motor power ratios based on strafe angle
        double frontLeftPower = Math.sin(robotAngle) + Math.cos(robotAngle);
        double frontRightPower = Math.sin(robotAngle) - Math.cos(robotAngle);
        double backLeftPower = Math.sin(robotAngle) - Math.cos(robotAngle);
        double backRightPower = Math.sin(robotAngle) + Math.cos(robotAngle);

        // Normalize the motor powers to prevent any value from exceeding 1.0
        double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
        if (maxPower > 1.0) {
            frontLeftPower /= maxPower;
            frontRightPower /= maxPower;
            backLeftPower /= maxPower;
            backRightPower /= maxPower;
        }

        // Set target positions for the motors using the encoder
        int frontLeftTarget = frontLeft.getCurrentPosition() + (int)(frontLeftPower * targetPosition);
        int frontRightTarget = frontRight.getCurrentPosition() + (int)(frontRightPower * targetPosition);
        int backLeftTarget = backLeft.getCurrentPosition() + (int)(backLeftPower * targetPosition);
        int backRightTarget = backRight.getCurrentPosition() + (int)(backRightPower * targetPosition);

        frontLeft.setTargetPosition(frontLeftTarget);
        frontRight.setTargetPosition(frontRightTarget);
        backLeft.setTargetPosition(backLeftTarget);
        backRight.setTargetPosition(backRightTarget);

        // Set motors to run to the target positions
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the runtime to track timeout
        runtime.reset();

        // Run the motors until the target position is reached or timeout occurs
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeout)) {

            // Calculate the error between the target and current position for each motor
            int frontLeftError = Math.abs(frontLeft.getTargetPosition() - frontLeft.getCurrentPosition());
            int frontRightError = Math.abs(frontRight.getTargetPosition() - frontRight.getCurrentPosition());
            int backLeftError = Math.abs(backLeft.getTargetPosition() - backLeft.getCurrentPosition());
            int backRightError = Math.abs(backRight.getTargetPosition() - backRight.getCurrentPosition());

            // Exit loop if all motor errors are less than 3 ticks
            if (frontLeftError < 3 && frontRightError < 3 && backLeftError < 3 && backRightError < 3) {
                break;
            }

            // Set motor powers while strafing
            frontLeft.setPower(frontLeftPower * power);
            frontRight.setPower(frontRightPower * power);
            backLeft.setPower(backLeftPower * power);
            backRight.setPower(backRightPower * power);

            // Add telemetry to show the errors for each motor
            opMode.telemetry.addData("FL Error", frontLeftError);
            opMode.telemetry.addData("FR Error", frontRightError);
            opMode.telemetry.addData("BL Error", backLeftError);
            opMode.telemetry.addData("BR Error", backRightError);
            opMode.telemetry.update();
        }

        // Stop all motors when done
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Reset the motors to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Strafes the robot in a specified direction for a given distance while rotating to a target heading.
     *
     * @param distance The distance in millimeters the robot should strafe.
     * @param angle The angle in degrees at which to strafe relative to the robot's front.
     *              0 degrees means strafing directly to the right, 90 degrees means forward.
     * @param power The speed at which the robot should move (range 0.0 to 1.0).
     * @param timeout The maximum time in seconds to complete the strafe.
     * @param targetHeading The target heading in radians that the robot should rotate to by the end of the trajectory.
     */
    public void strafeWithHeading(double distance, double angle, double power, double timeout, double targetHeading) {
        // Convert distance in mm to target encoder counts
        final double TICKS_PER_MM = TICKS_PER_REV / WHEEL_CIRCUMFERENCE_MM;
        int targetPosition = (int)(distance * TICKS_PER_MM);

        // Convert angle to radians for calculations
        double robotAngle = Math.toRadians(angle);

        // Get the initial position and heading of the robot using the odometry pods
        odo.update();
        Pose2D initialPose = odo.getPosition();
        double initialHeading = initialPose.getHeading(AngleUnit.RADIANS);  // Heading in radians
        double initialX = initialPose.getX(DistanceUnit.MM);  // X position in mm
        double initialY = initialPose.getY(DistanceUnit.MM);  // Y position in mm

        // Calculate motor powers for strafing at the specified angle
        double v1 = Math.sin(robotAngle) - Math.cos(robotAngle); // frontLeft
        double v2 = Math.sin(robotAngle) + Math.cos(robotAngle); // frontRight
        double v3 = Math.sin(robotAngle) + Math.cos(robotAngle); // backLeft
        double v4 = Math.sin(robotAngle) - Math.cos(robotAngle); // backRight

        // Set target positions for the motors using the encoder
        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + targetPosition);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() - targetPosition);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - targetPosition);
        backRight.setTargetPosition(backRight.getCurrentPosition() + targetPosition);

        // Set motors to run to the target positions
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Reset the runtime to track timeout
        runtime.reset();

        // Run the motors until the target position is reached or timeout occurs
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < timeout) &&
                (frontLeft.isBusy() && frontRight.isBusy() && backLeft.isBusy() && backRight.isBusy())) {

            // Update odometry data
            odo.update();
            Pose2D currentPose = odo.getPosition();
            double currentHeading = initialPose.getHeading(AngleUnit.RADIANS);  // Heading in radians
            double currentX = initialPose.getX(DistanceUnit.MM);  // X position in mm
            double currentY = initialPose.getY(DistanceUnit.MM);  // Current Y position in mm

            // Calculate heading error (the difference between current and target heading)
            double headingError = targetHeading - currentHeading;

            // Adjust motor power to correct for heading change over time
            double headingCorrection = Range.clip(headingError * 0.1, -0.1, 0.1);  // Adjust this constant for how aggressive rotation is

            // Set motor powers while staying within [-1.0, 1.0] range and applying the heading correction
            frontLeft.setPower(Range.clip((v1 * power) - headingCorrection, -1.0, 1.0));
            frontRight.setPower(Range.clip((v2 * power) + headingCorrection, -1.0, 1.0));
            backLeft.setPower(Range.clip((v3 * power) - headingCorrection, -1.0, 1.0));
            backRight.setPower(Range.clip((v4 * power) + headingCorrection, -1.0, 1.0));

            // Optionally, add telemetry for monitoring progress
            opMode.telemetry.addData("Path", "Driving to %7d :%7d", targetPosition, targetPosition);
            opMode.telemetry.addData("Heading", "Initial: %.2f, Current: %.2f, Target: %.2f", Math.toDegrees(initialHeading), Math.toDegrees(currentHeading), Math.toDegrees(targetHeading));
            opMode.telemetry.addData("Position", "X: %.2f mm, Y: %.2f mm", currentX, currentY);
            opMode.telemetry.update();
        }

        // Stop all motors when done
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Reset the motors to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
