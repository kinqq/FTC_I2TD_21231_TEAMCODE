package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.util.Constants;

@Autonomous(group = "Auto")
@Config
public class Auto extends LinearOpMode {
    enum Alliance {
        BLUE,
        RED
    }

    enum Position {
        LEFT,
        RIGHT
    }

    public Alliance alliance = Alliance.BLUE;
    public Position position = Position.LEFT;

    @Override
    public void runOpMode() {
        AutoUtil auto = new AutoUtil(this);
        auto.grabber(Constants.GRABBER_CLOSE);
        auto.pitch(Constants.PITCH_FORWARD);
        auto.roll(Constants.ROLL_0);

        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.left_bumper) {
                alliance = Alliance.BLUE;
            }
            if (gamepad1.right_bumper) {
                alliance = Alliance.RED;
            }
            if (gamepad1.left_trigger > 0.25) {
                position = Position.LEFT;
            }
            if (gamepad1.right_trigger > 0.25) {
                position = Position.RIGHT;
            }
            if (gamepad2.left_bumper) {
                auto.grabber(Constants.GRABBER_CLOSE);
            }
            if (gamepad2.right_bumper) {
                auto.grabber(Constants.GRABBER_OPEN);
            }
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Position", position);telemetry.addData("Target X", 0);
            telemetry.addData("Target Y", 0);
            telemetry.addData("Current X", 0);
            telemetry.addData("Current Y", 0);
            telemetry.addData("Power X", 0);
            telemetry.addData("Power Y", 0);
            telemetry.addData("Error X", 0);
            telemetry.addData("Error Y", 0);
            telemetry.addData("Distance to Target", 0);
            telemetry.update();
        }

//        if (position == Position.LEFT) {
//            auto.strafe(1350, -140, 0.4, 5);
//            auto.elevator(Constants.ELE_CHAMBER_HIGH, 1, 5);
//            auto.strafe(-150, 90, 0.2, 2);
//            auto.elevator(Constants.ELE_CHAMBER_HIGH_DROP, 1, 1.5);
//            sleep(500);
//            auto.grabber(Constants.GRABBER_OPEN);
//            sleep(500);
//            auto.elevator(Constants.ELE_BOT, 1, 2.5);
//            auto.strafe(-2000, 0, 0.5, 7); //TODO: change this distance
//            auto.strafe(600, 90, 0.5, 5);
//        }
//        //TODO: Not tested
//        else if (position == Position.RIGHT) {
//            auto.strafe(700, -80, 0.4, 5);
//            auto.elevator(Constants.ELE_CHAMBER_HIGH, 1, 5);
//            auto.strafe(-150, 90, 0.2, 2);
//            auto.elevator(Constants.ELE_CHAMBER_HIGH_DROP, 1, 1.5);
//            sleep(500);
//            auto.grabber(Constants.GRABBER_OPEN);
//            sleep(500);
//            auto.elevator(Constants.ELE_BOT, 1, 2.5);
//            auto.strafe(-1400, 0, 0.5, 7); //TODO: change this distance
//            auto.strafe(600, 90, 0.5, 5);
//        }


        if (position == Position.LEFT) {
            auto.pitch(Constants.PITCH_BACKWARD);
            auto.rotate(Constants.ROT_UP, 0.3, 1.5);
            auto.roll(Constants.ROLL_180);
            auto.strafeOdo(920, 225, 0.6, 5);
            auto.elevator(Constants.ELE_CHAMBER_HIGH, 1, 3);
            auto.strafeOdo(80, -90, 0.3, 2);
            sleep(800);
            auto.elevator(Constants.ELE_CHAMBER_HIGH_DROP, 1, 2);
            sleep(500);
            auto.grabber(Constants.GRABBER_OPEN);
            sleep(300);
            auto.elevator(Constants.ELE_BOT, 1, 2);
            auto.grabber(Constants.GRABBER_CLOSE);

            auto.strafeOdo(1000, 13, 0.6, 4);
            auto.pitch(Constants.PITCH_FORWARD);
            auto.turn(180, 0.7, 4);

//            auto.roll(Constants.ROLL_0);
            auto.grabber(Constants.GRABBER_OPEN);
            auto.rotate(Constants.ROT_DOWN, 0.3, 2.5);
            auto.strafe(140, 90, 0.3, 2);

            auto.grabber(Constants.GRABBER_CLOSE);
            sleep(500);
            auto.rotate(Constants.ROT_UP, 0.5, 2);
            auto.strafe(840, -55, 0.6, 3);
            auto.turn(-45, 0.6, 2);
            auto.pitch(Constants.PITCH_UP);
            auto.elevator(Constants.ELE_BASKET_HIGH, 1, 4);
            sleep(500);

            auto.pitch(Constants.PITCH_BACKWARD);
            sleep(300);
            auto.grabber(Constants.GRABBER_OPEN);

            sleep(600);
            auto.pitch(Constants.PITCH_UP);
            auto.elevator(Constants.ELE_BOT, 1, 4);
            auto.pitch(Constants.PITCH_GRAB);
        }

        if (position == Position.RIGHT) {
            auto.driveToPosition(30, 30, 0);
            sleep(3000);
            auto.driveToPosition(30, 90, 0);
            sleep(3000);
            auto.driveToPosition(-30, 90, 0);
        }

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }

}