package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
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
            telemetry.addData("Alliance", alliance);
            telemetry.addData("Position", position);
            telemetry.update();
        }

        auto.strafe(400, -120, 0.4, 5);
        auto.elevator(Constants.ELE_LOW, 1, 5);
        auto.strafe(-30, 90, 0.2, 2);
        auto.elevator(Constants.ELE_DROP_SPECIMEN, 1, 5);
        sleep(1000);
        auto.grabber(Constants.GRABBER_OPEN);
        auto.strafe(200, 90, 0.4, 5);

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}