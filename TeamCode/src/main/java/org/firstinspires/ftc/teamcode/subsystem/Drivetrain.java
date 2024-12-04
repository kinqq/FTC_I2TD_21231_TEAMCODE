package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.roadrunner.ftc.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.DcMotorGroup;

public class Drivetrain {
    private DcMotorEx _frontLeftMotor, _frontRightMotor, _backLeftMotor, _backRightMotor;
    private DcMotorGroup dtMotors;
    static public GoBildaPinpointDriver odo;
    private double maxPower;
    private boolean isFieldCentricDriveEnabled = true;

    public Drivetrain(HardwareMap hwMap) {
        _frontLeftMotor = hwMap.get(DcMotorEx.class, "leftFront");
        _frontRightMotor = hwMap.get(DcMotorEx.class, "rightFront");
        _backLeftMotor = hwMap.get(DcMotorEx.class, "leftBack");
        _backRightMotor = hwMap.get(DcMotorEx.class, "rightBack");

        _frontLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);
        _backLeftMotor.setDirection(DcMotorEx.Direction.REVERSE);

        odo = hwMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(0, 0);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        dtMotors = new DcMotorGroup(_frontLeftMotor, _frontRightMotor, _backLeftMotor, _backRightMotor);
        dtMotors.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double strafeSpeed,
                      double forwardSpeed,
                      double turnSpeed) {
        odo.update();

        double heading;
        if (!isFieldCentricDriveEnabled) heading = 0;
        else heading = -odo.getHeading();

        forwardSpeed *= -1;

        double xSpeed = strafeSpeed * Math.cos(heading) - forwardSpeed * Math.sin(heading);
        double ySpeed = strafeSpeed * Math.sin(heading) + forwardSpeed * Math.cos(heading);

        // Counteract imperfect strafing
        xSpeed *= 1.1;

        double max = Math.max(Math.abs(ySpeed) + Math.abs(xSpeed) + Math.abs(turnSpeed), maxPower);

        _frontLeftMotor.setPower((ySpeed + xSpeed + turnSpeed) / max);
        _frontRightMotor.setPower((ySpeed - xSpeed - turnSpeed) / max);
        _backLeftMotor.setPower((ySpeed - xSpeed + turnSpeed) / max);
        _backRightMotor.setPower((ySpeed + xSpeed - turnSpeed) / max);
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
}
