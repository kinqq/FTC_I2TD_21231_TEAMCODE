package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.DcMotorGroup;

public class Drivetrain {
    @Config
    public static class headingPIDF {
        public static double p = 2.2, i = 0, d = 0, f = 0;
    }

    private DcMotorEx _frontLeftMotor, _frontRightMotor, _backLeftMotor, _backRightMotor;
    private DcMotorGroup dtMotors;
    static public GoBildaPinpointDriver odo;
    private double maxPower;
    private boolean isFieldCentricDriveEnabled = true;
    private boolean isHeadingLockEnabled = true;
    public double targetHeading;
    private boolean isTurning = false;
    private PIDFController headingPIDFController;

    public Drivetrain(HardwareMap hwMap) {
        _frontLeftMotor = hwMap.get(DcMotorEx.class, "leftFront");
        _frontRightMotor = hwMap.get(DcMotorEx.class, "rightFront");
        _backLeftMotor = hwMap.get(DcMotorEx.class, "leftBack");
        _backRightMotor = hwMap.get(DcMotorEx.class, "rightBack");

        _frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        _backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(-3.45, -3.65);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
//        odo.resetPosAndIMU();

        dtMotors = new DcMotorGroup(_frontLeftMotor, _frontRightMotor, _backLeftMotor, _backRightMotor);
        dtMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        dtMotors.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        headingPIDFController = new PIDFController(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);
        targetHeading = -odo.getHeading();
    }

    private double smoothControl(double value) {
        return 0.5 * Math.tan(1.107 * value);
    }

    public void drive(double strafeSpeed,
                      double forwardSpeed,
                      double turnSpeed) {
        odo.update();
        headingPIDFController.setPIDF(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);

        strafeSpeed = smoothControl(strafeSpeed);
        forwardSpeed = smoothControl(forwardSpeed);
        turnSpeed = smoothControl(turnSpeed);

        double heading;
        if (!isFieldCentricDriveEnabled) heading = 0;
        else heading = -odo.getHeading();

        if (turnSpeed != 0) isTurning = true;
        else if (isTurning && Math.abs(odo.getHeadingVelocity()) < 0.02) {
            isTurning = false;
            targetHeading = heading;
        }

        if (!isTurning && isHeadingLockEnabled)
            turnSpeed = headingPIDFController.calculate(heading, targetHeading);

        forwardSpeed *= -1;

        double xSpeed = strafeSpeed * Math.cos(heading) - forwardSpeed * Math.sin(heading);
        double ySpeed = strafeSpeed * Math.sin(heading) + forwardSpeed * Math.cos(heading);

        // Counteract imperfect strafing
        xSpeed *= 1.1;

        double max = Math.max(Math.abs(ySpeed) + Math.abs(xSpeed) + Math.abs(turnSpeed), 1.0);

        _frontLeftMotor.setPower((ySpeed + xSpeed + turnSpeed) / max * maxPower);
        _frontRightMotor.setPower((ySpeed - xSpeed - turnSpeed) / max * maxPower);
        _backLeftMotor.setPower((ySpeed - xSpeed + turnSpeed) / max * maxPower);
        _backRightMotor.setPower((ySpeed + xSpeed - turnSpeed) / max * maxPower);
    }

    public void toggleHeadingLock() {
        isHeadingLockEnabled = !isHeadingLockEnabled;
        targetHeading = -odo.getHeading();
    }

    public void resetImu() {
        odo.resetPosAndIMU();
    }

    public void setMaxPower(double maxPower) {
        this.maxPower = maxPower;
    }

    public void toggleFieldCentricDrive() {
        isFieldCentricDriveEnabled ^= true;
    }

    public Pose2D getPosition() {
        return odo.getPosition();
    }

    public boolean isFieldCentricDriveEnabled() {
        return isFieldCentricDriveEnabled;
    }

    public boolean isHeadingLockEnabled() {
        return isHeadingLockEnabled;
    }
}
