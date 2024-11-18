package org.firstinspires.ftc.teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.util.Constants.angleCoeff;
import static org.firstinspires.ftc.teamcode.util.Constants.powerCoeff;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.util.LED;

public class AutoUtil {
    LinearOpMode opMode;
    static ElapsedTime runtime = new ElapsedTime();

    // Declare motor variables
    DcMotorEx frontLeft;
    DcMotorEx frontRight;
    DcMotorEx backLeft;
    DcMotorEx backRight;
    DcMotor eleLeft, eleRight, rotLeft, rotRight;
    Servo grabber, pitch, roll;
//    LED led;

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
        opMode.telemetry = new MultipleTelemetry(linearOpMode.telemetry, FtcDashboard.getInstance().getTelemetry());

        this.frontLeft = linearOpMode.hardwareMap.get(DcMotorEx.class, "leftFront");
        this.frontRight = linearOpMode.hardwareMap.get(DcMotorEx.class, "rightFront");
        this.backLeft = linearOpMode.hardwareMap.get(DcMotorEx.class, "leftBack");
        this.backRight = linearOpMode.hardwareMap.get(DcMotorEx.class, "rightBack");

//        this.frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        this.backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        this.frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        this.backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        this.eleLeft =  linearOpMode.hardwareMap.get(DcMotorEx.class, "leftEle");
        this.eleRight =  linearOpMode.hardwareMap.get(DcMotorEx.class, "rightEle");
        this.rotLeft =  linearOpMode.hardwareMap.get(DcMotorEx.class, "leftRot");
        this.rotRight =  linearOpMode.hardwareMap.get(DcMotorEx.class, "rightRot");

        this.grabber = linearOpMode.hardwareMap.get(Servo.class, "grabber");
        this.pitch = linearOpMode.hardwareMap.get(Servo.class, "pitch");
        this.roll = linearOpMode.hardwareMap.get(Servo.class, "roll");

        eleLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        eleRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        eleLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        eleLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        eleRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rotLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rotLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rotRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.odo = linearOpMode.hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(0, -6.5);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();
//        odo.setPosition(new Pose2D(DistanceUnit.MM, 0, 0, AngleUnit.DEGREES, 0));
    }

//    public void resetOdo() {
//        odo.resetPosAndIMU();
//    }

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

            if (eleLeftDiff < 10 || eleRightDiff < 10) {
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
        rotLeft.setTargetPosition(position);
        rotLeft.setPower(power);
        rotLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rotRight.setTargetPosition(position);
        rotRight.setPower(power);
        rotRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        while ((opMode.opModeIsActive()) && runtime.seconds() < timeout) {
            int rotLeftDiff = Math.abs(rotLeft.getTargetPosition() - rotLeft.getCurrentPosition());

            if (rotLeftDiff < 10) {
                break;
            }
        }

        rotLeft.setPower(0);
        rotRight.setPower(0);
    }

    public void pitch(double pos) {
        pitch.setPosition(pos);
    }

    public void roll(double pos) {
        roll.setPosition(pos);
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
            if (frontLeftError < 10 && frontRightError < 10 && backLeftError < 10 && backRightError < 10) {
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

    public void forwardOdo(double distance) {
        PIDFController pid = new PIDFController(0.03, 0, 0, 0);
        pid.setTolerance(0.1);

        odo.update();
        double firstPos = odo.getPosY();

        pid.reset();
        pid.setSetPoint(distance);

        do {
            double pos = odo.getPosY();
            double power = Range.clip(pid.calculate(pos - firstPos), -1, 1);

            frontRight.setPower(power);
            frontLeft.setPower(power);
            backRight.setPower(power);
            backLeft.setPower(power);

        } while (opMode.opModeIsActive() && !opMode.isStopRequested() && !pid.atSetPoint());
    }

    public void strafeOdo(double distance, double angle, double power, double timeout) {
        // Convert angle to radians
        double robotAngle = Math.toRadians(angle);
        odo.update();

        double initialHeading = odo.getHeading();

        // Calculate target X and Y in odometry space
        Pose2D currentPosition = odo.getPosition();
        double targetX = currentPosition.getY(DistanceUnit.MM) + distance * Math.cos(robotAngle);
        double targetY = currentPosition.getX(DistanceUnit.MM) + distance * Math.sin(robotAngle);

        // Reset timer for timeout
        runtime.reset();

        while (opMode.opModeIsActive() && runtime.seconds() < timeout) {
            odo.update(); // Update odometry position

            // Get current position and heading
            currentPosition = odo.getPosition();
            double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);
            double currentX = currentPosition.getY(DistanceUnit.MM);
            double currentY = currentPosition.getX(DistanceUnit.MM);

            // Calculate positional error
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            // Calculate the distance to target
            double distanceToTarget = Math.hypot(errorX, errorY);

            double headingError = initialHeading - currentHeading;
            headingError = (headingError + Math.PI) % (2 * Math.PI) - Math.PI;

            // If close enough to target, break
            if (distanceToTarget < 10 && Math.abs(headingError) < Math.toRadians(2)) { // Tolerance of 15mm
                break;
            }

            // Use adjusted trigonometric values for drive power
            double drivePowerX = (-errorX / (distanceToTarget + 15)) * power;
            double drivePowerY = (errorY / (distanceToTarget + 15)) * power;

            // Use IMU for heading correction if available
            double correction = (headingError / Math.toRadians(45)) * power * angleCoeff;

            // Apply motor powers with correction
            double frontLeftPower = drivePowerY + drivePowerX + correction;
            double frontRightPower = drivePowerY - drivePowerX - correction;
            double backLeftPower = drivePowerY - drivePowerX + correction;
            double backRightPower = drivePowerY + drivePowerX - correction;

            // Normalize powers to ensure no value exceeds 1.0
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Telemetry for monitoring
            opMode.telemetry.addData("Target X", targetX);
            opMode.telemetry.addData("Target Y", targetY);
            opMode.telemetry.addData("Current X", currentX);
            opMode.telemetry.addData("Current Y", currentY);
            opMode.telemetry.addData("Power X", drivePowerX);
            opMode.telemetry.addData("Power Y", drivePowerY);
            opMode.telemetry.addData("Error X", errorX);
            opMode.telemetry.addData("Error Y", errorY);
            opMode.telemetry.addData("Error H", headingError);
            opMode.telemetry.addData("Distance to Target", distanceToTarget);
            opMode.telemetry.update();
        }

        // Stop all motors once target is reached or timeout occurs
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Reset odometry to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeXY(double x, double y, double power, double timeout) {
        // Convert angle to radians
        odo.update();

        // Calculate target X and Y in odometry space
        Pose2D currentPosition = odo.getPosition();
        double targetX = currentPosition.getY(DistanceUnit.MM) + x;
        double targetY = currentPosition.getX(DistanceUnit.MM) + y;

        // Reset timer for timeout
        runtime.reset();

        while (opMode.opModeIsActive() && runtime.seconds() < timeout) {
            odo.update(); // Update odometry position

            // Get current position and heading
            currentPosition = odo.getPosition();
            double currentX = currentPosition.getY(DistanceUnit.MM);
            double currentY = currentPosition.getX(DistanceUnit.MM);

            // Calculate positional error
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            // Calculate the distance to target
            double distanceToTarget = Math.hypot(errorX, errorY);

            // If close enough to target, break
            if (distanceToTarget < 10 && Math.abs(odo.getHeading() - 0) < 0.01) { // Tolerance of 10mm
                break;
            }

            // Calculate drive powers proportional to error
            double drivePowerX = (-errorX / (distanceToTarget + 5)) * power * powerCoeff;
            double drivePowerY = (errorY / (distanceToTarget + 5)) * power * powerCoeff;

            // Use IMU for heading correction if available
            double correction = -odo.getPosition().getHeading(AngleUnit.RADIANS) * angleCoeff;

            // Apply motor powers with correction
            double frontLeftPower = drivePowerY + drivePowerX + correction;
            double frontRightPower = drivePowerY - drivePowerX - correction;
            double backLeftPower = drivePowerY - drivePowerX + correction;
            double backRightPower = drivePowerY + drivePowerX - correction;

            // Normalize powers to ensure no value exceeds 1.0
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Telemetry for monitoring
            opMode.telemetry.addData("Target X", targetX);
            opMode.telemetry.addData("Target Y", targetY);
            opMode.telemetry.addData("Current X", currentX);
            opMode.telemetry.addData("Current Y", currentY);
            opMode.telemetry.addData("Power X", drivePowerX);
            opMode.telemetry.addData("Power Y", drivePowerY);
            opMode.telemetry.addData("Error X", errorX);
            opMode.telemetry.addData("Error Y", errorY);
            opMode.telemetry.addData("Distance to Target", distanceToTarget);
            opMode.telemetry.update();
        }

        // Stop all motors once target is reached or timeout occurs
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Reset odometry to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void strafeTurn(double distance, double angle, double power, double timeout, double finalHeading) {
        // Convert target angle to radians
        double targetAngle = Math.toRadians(angle);
        odo.update();

        // Calculate target X and Y in odometry space
        Pose2D currentPosition = odo.getPosition();
        double targetX = currentPosition.getY(DistanceUnit.MM) + distance * Math.cos(targetAngle);
        double targetY = currentPosition.getX(DistanceUnit.MM) + distance * Math.sin(targetAngle);

        // Reset timer for timeout
        runtime.reset();

        while (opMode.opModeIsActive() && runtime.seconds() < timeout) {
            odo.update(); // Update odometry position

            // Get current position and heading
            currentPosition = odo.getPosition();
            double currentX = currentPosition.getY(DistanceUnit.MM);
            double currentY = currentPosition.getX(DistanceUnit.MM);
            double currentHeading = currentPosition.getHeading(AngleUnit.RADIANS);

            // Calculate positional error
            double errorX = targetX - currentX;
            double errorY = targetY - currentY;

            // Calculate distance to the target
            double distanceToTarget = Math.hypot(errorX, errorY);

            // Calculate heading error
            double headingError = Math.toRadians(finalHeading) - currentHeading;

            // Normalize heading error to range [-PI, PI]
            headingError = (headingError + Math.PI) % (2 * Math.PI) - Math.PI;

            // If close enough to target position and heading, break
            if (distanceToTarget < 10 && Math.abs(headingError) < Math.toRadians(1)) { // Tolerance of 10mm and 1 degree
                break;
            }

            // Adjust powers based on current heading
            double adjustedAngle = targetAngle - currentHeading;
            double adjustedCos = Math.cos(adjustedAngle);
            double adjustedSin = Math.sin(adjustedAngle);

            // Calculate drive powers proportional to error, adjusted for current heading
            double drivePowerX = (-errorX / (distanceToTarget + 15)) * power * adjustedCos * powerCoeff;
            double drivePowerY = (errorY / (distanceToTarget + 15)) * power * adjustedSin * powerCoeff;

            // Calculate turning power proportional to heading error
            double turnPower = (headingError / Math.toRadians(45)) * power * 0.5; // Scale turn within ±45 degrees

            // Apply motor powers for combined strafing and turning
            double frontLeftPower = drivePowerY + drivePowerX + turnPower;
            double frontRightPower = drivePowerY - drivePowerX - turnPower;
            double backLeftPower = drivePowerY - drivePowerX + turnPower;
            double backRightPower = drivePowerY + drivePowerX - turnPower;

            // Normalize powers to ensure no value exceeds 1.0
            double maxPower = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));
            if (maxPower > 1.0) {
                frontLeftPower /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower /= maxPower;
                backRightPower /= maxPower;
            }

            // Set motor powers
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            // Telemetry for debugging
            opMode.telemetry.addData("Target X", targetX);
            opMode.telemetry.addData("Target Y", targetY);
            opMode.telemetry.addData("Current X", currentX);
            opMode.telemetry.addData("Current Y", currentY);
            opMode.telemetry.addData("Heading Error (deg)", Math.toDegrees(headingError));
            opMode.telemetry.addData("Turn Power", turnPower);
            opMode.telemetry.addData("Distance to Target", distanceToTarget);
            opMode.telemetry.update();
        }

        // Stop all motors once target is reached or timeout occurs
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Reset odometry to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turn(double targetHeading, double power, double timeout) {
        odo.update(); // Update odometry

        targetHeading = targetHeading + Math.toDegrees(odo.getHeading());
        // Normalize the target heading to range [-180, 180]
        targetHeading = (targetHeading + 360) % 360; // Normalize to [0, 360]
        if (targetHeading > 180) {
            targetHeading -= 360; // Normalize to [-180, 180]
        }

        // Reset runtime
        runtime.reset();

        while (opMode.opModeIsActive() && runtime.seconds() < timeout) {
            odo.update(); // Update odometry

            // Get current heading in degrees
            double currentHeading = Math.toDegrees(odo.getHeading());

            // Normalize current heading to [-180, 180]
            currentHeading = (currentHeading + 360) % 360;
            if (currentHeading > 180) {
                currentHeading -= 360;
            }

            // Calculate the heading error
            double headingError = targetHeading - currentHeading;

            // Normalize the heading error to [-180, 180]
            headingError = (headingError + 360) % 360;
            if (headingError > 180) {
                headingError -= 360;
            }

            // Break if the heading is within tolerance (e.g., 1 degree)
            if (Math.abs(headingError) < 1) { // 1 degree tolerance
                break;
            }

            // Calculate turn power proportional to heading error
            double turnPower = (headingError / 45) * power; // Scale max turn power for ±45 degrees

            // Clamp turn power to [-power, power]
            turnPower = Math.max(-power, Math.min(turnPower, power));

            // Apply turning power to motors
            frontLeft.setPower(turnPower);
            frontRight.setPower(-turnPower);
            backLeft.setPower(turnPower);
            backRight.setPower(-turnPower);

            // Telemetry for debugging
            opMode.telemetry.addData("Target Heading", targetHeading);
            opMode.telemetry.addData("Current Heading", currentHeading);
            opMode.telemetry.addData("Heading Error", headingError);
            opMode.telemetry.addData("Turn Power", turnPower);
            opMode.telemetry.update();
        }

        // Stop all motors once the turn is complete or timeout occurs
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        // Reset motors to run using encoders
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
