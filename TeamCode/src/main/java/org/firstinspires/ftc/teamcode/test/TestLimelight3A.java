/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode.test;

import static org.firstinspires.ftc.teamcode.util.Constants.ROLL_TICK_ON_ZERO;
import static org.firstinspires.ftc.teamcode.util.Constants.ROLL_TICK_PER_DEG;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Comparator;
import java.util.Iterator;
import java.util.List;
import java.util.ListIterator;
import java.util.Set;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp(name = "Sensor: Limelight3A", group = "Test")
public class TestLimelight3A extends LinearOpMode {
    private Limelight3A limelight;
    private ServoImplEx roll;

    List<Double> angleAvg = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        roll = (ServoImplEx) hardwareMap.get(Servo.class, "roll");
        roll.setPwmRange(new PwmControl.PwmRange(500, 2500));

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(),(int)status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            LLResult result = limelight.getLatestResult();

            if (result != null) {
                // Access general information
                double captureLatency = result.getCaptureLatency();
                double targetingLatency = result.getTargetingLatency();
                double parseLatency = result.getParseLatency();
                telemetry.addData("LL Latency", captureLatency + targetingLatency);
                telemetry.addData("Parse Latency", parseLatency);

                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
//                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
//                    telemetry.addData("tync", result.getTyNC());

                    // Access color results
                    List<LLResultTypes.ColorResult> colorResults = result.getColorResults();

                    LLResultTypes.ColorResult cr = colorResults.get(0);
                    List<List<Double>> targetCorners = cr.getTargetCorners();
                    List<List<Double>> ogcorners = cr.getTargetCorners();
                    List<List<Double>> corners = new ArrayList<>();

                    if (targetCorners.size() == 4)
                        corners = targetCorners;
                    else if (targetCorners.size() > 4){
                        // Sort by x
                        targetCorners.sort(Comparator.comparingDouble(point -> point.get(0)));
                        corners.add(targetCorners.get(0)); // smallest x
                        corners.add(targetCorners.get(targetCorners.size() - 1)); // smallest y

                        // Sort by y
                        targetCorners.sort(Comparator.comparingDouble(point -> point.get(1)));
                        corners.add(targetCorners.get(0)); // smallest y
                        corners.add(targetCorners.get(targetCorners.size() - 1)); // smallest y
                    }

                    if (!corners.isEmpty()) {
                        corners.sort(Comparator.comparingDouble(point -> point.get(1)));

                        List<Double> point1 = corners.get(0); // smallest y
                        List<Double> point2 = corners.get(1); // second smallest y

                        double theta;
                        if (point1.get(0) > point2.get(0)) {
                            theta = 1;
                        } else {
                            theta = -1;
                        }

                        // Calculate the angle in radians
                        theta *= Math.atan(Math.abs(point1.get(1) - point2.get(1)) / Math.abs(point1.get(0) - point2.get(0)));

                        // Using distance formula
                        double dist1 = Math.hypot(point1.get(0) - point2.get(0), point1.get(1) - point2.get(1));
                        int adjIdx = ogcorners.indexOf(point2);
                        ++adjIdx;
                        adjIdx %= 3;
                        List<Double> adjacentPoint = ogcorners.get(adjIdx);
                        double dist2 = Math.hypot(point2.get(0) - adjacentPoint.get(0), point2.get(1) - adjacentPoint.get(1));

//                            telemetry.addData("dist1", dist1);
//                            telemetry.addData("dist2", dist2);

                        if (Math.abs(dist1 - dist2) <= 50) {
                            theta += Math.PI / 2; // adding 90 degrees in radians
                        }

                        if (theta < 0) {
                            theta += Math.PI;
                        }

                        if (theta >= Math.PI / 2) {
                            theta -= Math.PI;
                        }

                        // Converting angle to degrees for better understanding
                        double angleInDegrees = Math.toDegrees(theta);
                        telemetry.addData("Angle", angleInDegrees);

                        double TICK = ROLL_TICK_ON_ZERO + ROLL_TICK_PER_DEG * angleInDegrees;
                        if (!Double.isNaN(angleInDegrees)) {
                            roll.setPosition(TICK);
                            angleAvg.add(angleInDegrees);
                        }

                        if (angleAvg.size() > 50) {
                            angleAvg.remove(0);
                        }

                        double avg = 0;
                        if (!angleAvg.isEmpty()) {
                            for (double angle : angleAvg) {
                                avg += angle;
                            }
                            avg /= angleAvg.size();
                        }
                        telemetry.addData("AngleAVG", avg);
                        telemetry.addLine(targetCorners.toString());
                        telemetry.addData("Size", targetCorners.size());
                        telemetry.addData("ColorY", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                        telemetry.addData("ColorN", "X: %.2f, Y: %.2f", cr.getTargetXDegreesNoCrosshair(), cr.getTargetYDegreesNoCrosshair());
                        telemetry.addData("Pixels", "X: %.2f, Y: %.2f", cr.getTargetXPixels(), cr.getTargetYPixels());
                    }
                }
            } else {
                telemetry.addData("Limelight", "No data available");
            }

            telemetry.update();
        }
        limelight.stop();
    }
}
