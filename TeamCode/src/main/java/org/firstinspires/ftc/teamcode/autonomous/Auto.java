package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.AutoUtil;
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
            telemetry.addData("Position", position);
            telemetry.update();
        }

        if (position == Position.LEFT) {
            auto.strafe(1350, -140, 0.4, 5);
            auto.elevator(Constants.ELE_HIGH_CHAMBER, 1, 5);
            auto.strafe(-150, 90, 0.2, 2);
            auto.elevator(Constants.ELE_DROP_SPECIMEN, 1, 1.5);
            sleep(500);
            auto.grabber(Constants.GRABBER_OPEN);
            sleep(500);
            auto.elevator(Constants.ELE_BOT, 1, 2.5);
            auto.strafe(-2000, 0, 0.5, 7); //TODO: change this distance
            auto.strafe(600, 90, 0.5, 5);
        }
        //TODO: Not tested
        else if (position == Position.RIGHT) {
            auto.strafe(700, -80, 0.4, 5);
            auto.elevator(Constants.ELE_HIGH_CHAMBER, 1, 5);
            auto.strafe(-150, 90, 0.2, 2);
            auto.elevator(Constants.ELE_DROP_SPECIMEN, 1, 1.5);
            sleep(500);
            auto.grabber(Constants.GRABBER_OPEN);
            sleep(500);
            auto.elevator(Constants.ELE_BOT, 1, 2.5);
            auto.strafe(-1400, 0, 0.5, 7); //TODO: change this distance
            auto.strafe(600, 90, 0.5, 5);
        }



        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}