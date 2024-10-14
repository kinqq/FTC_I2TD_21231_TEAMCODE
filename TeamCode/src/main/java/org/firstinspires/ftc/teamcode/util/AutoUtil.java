package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriverRR;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class AutoUtil {
    LinearOpMode opMode;
    static ElapsedTime runtime = new ElapsedTime();

    // Declare motor variables
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    // Odometry driver
    GoBildaPinpointDriverRR odo;

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

        this.frontLeft = linearOpMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.frontRight = linearOpMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.backLeft = linearOpMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.backRight = linearOpMode.hardwareMap.get(DcMotorEx.class, "frontLeft");
        this.odo = linearOpMode.hardwareMap.get(GoBildaPinpointDriverRR.class, "odo");
    }

    /**
     * Strafes the robot in a specified direction for a given distance while maintaining its heading using odometry.
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
        int targetPosition = (int)(distance * TICKS_PER_MM);

        // Convert angle to radians for calculations
        double robotAngle = Math.toRadians(angle);

        // Get the initial heading of the robot using the odometry pods
        odo.update();
        Pose2d initialPose = odo.getPositionRR();
        double initialHeading = initialPose.heading.toDouble();  // Heading in radians

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
            Pose2d currentPose = odo.getPositionRR();
            double currentHeading = currentPose.heading.toDouble();  // Heading in radians
            double headingError = initialHeading - currentHeading;

            // Adjust motor power to correct for heading deviation
            double headingCorrection = Range.clip(headingError * 0.1, -0.1, 0.1);

            // Set motor powers while staying within [-1.0, 1.0] range
            frontLeft.setPower(Range.clip((v1 * power) - headingCorrection, -1.0, 1.0));
            frontRight.setPower(Range.clip((v2 * power) + headingCorrection, -1.0, 1.0));
            backLeft.setPower(Range.clip((v3 * power) - headingCorrection, -1.0, 1.0));
            backRight.setPower(Range.clip((v4 * power) + headingCorrection, -1.0, 1.0));

            // Optionally, add telemetry for monitoring progress
            opMode.telemetry.addData("Path", "Driving to %7d :%7d", targetPosition, targetPosition);
            opMode.telemetry.addData("Heading", "Initial: %.2f, Current: %.2f", Math.toDegrees(initialHeading), Math.toDegrees(currentHeading));
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
        Pose2d initialPose = odo.getPositionRR();
        double initialHeading = initialPose.heading.toDouble();  // Heading in radians
        double initialX = initialPose.position.x;  // X position in mm
        double initialY = initialPose.position.y;  // Y position in mm

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
            Pose2d currentPose = odo.getPositionRR();
            double currentHeading = currentPose.heading.toDouble();  // Current heading in radians
            double currentX = currentPose.position.x;  // Current X position in mm
            double currentY = currentPose.position.y;  // Current Y position in mm

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
