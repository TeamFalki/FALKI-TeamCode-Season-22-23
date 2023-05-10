/*
 * Copyright (c) 2021 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.firstinspires.ftc.teamcode.Core.AprilTagDetectionPipeline;

import java.util.ArrayList;

@Autonomous(name = "PROJECT_FALCON", group = "NevermoreAuton")
public class PROJECT_FALCON extends LinearOpMode {
    private DcMotor linksHinten;
    private DcMotor linksVorne;
    private DcMotor rechtsHinten;
    private DcMotor rechtsVorne;
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int SIGNAL_TAG_1 = 18; // Tag ID 18 from the 36h11 family
    int SIGNAL_TAG_2 = 18;
    int SIGNAL_TAG_3 = 18;

    AprilTagDetection tagOfInterest = null;

    boolean tag1Found = false;
    boolean tag2Found = false;
    boolean tag3Found = false;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        linksHinten = hardwareMap.get(DcMotor.class, "linksHinten");
        linksVorne = hardwareMap.get(DcMotor.class, "linksVorne");
        rechtsHinten = hardwareMap.get(DcMotor.class, "rechtsHinten");
        rechtsVorne = hardwareMap.get(DcMotor.class, "rechtsVorne");

        linksVorne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linksVorne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        linksHinten.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linksHinten.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rechtsVorne.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rechtsVorne.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rechtsHinten.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rechtsHinten.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        linksHinten.setDirection(DcMotorSimple.Direction.FORWARD);
        linksVorne.setDirection(DcMotorSimple.Direction.FORWARD);
        rechtsHinten.setDirection(DcMotorSimple.Direction.REVERSE);
        rechtsVorne.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested()) {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if (currentDetections.size() != 0) {
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == SIGNAL_TAG_1) {
                        tagOfInterest = tag;
                        tag1Found = true;
                        tag2Found = false;
                        tag3Found = false;
                        break;
                    }

                    if (tag.id == SIGNAL_TAG_2) {
                        tagOfInterest = tag;
                        tag1Found = false;
                        tag2Found = true;
                        tag3Found = false;
                        break;
                    }

                    if (tag.id == SIGNAL_TAG_3) {
                        tagOfInterest = tag;
                        tag1Found = false;
                        tag2Found = false;
                        tag3Found = true;
                        break;
                    }
                }

                if (tag1Found) {
                    telemetry.addLine("Tag 1 found!");
                } else if (tag2Found) {
                    telemetry.addLine("Tag 2 found!");
                } else if (tag3Found) {
                    telemetry.addLine("Tag 3 found!");
                }

            } else {
                telemetry.addLine("Don't see tag of interest :(");

                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("But we HAVE seen the tag before");
                }

                telemetry.update();
                sleep(20);
            }
        }
        //START
        if (!tag1Found) {
            drive_lr(-0.5);
            sleep(1500);
            drive_lr(0);
            drive_fb(0.5);
            sleep(2500);
            drive_fb(0);
        }
        if (tag2Found) {
            drive_fb(0.5);
            sleep(3000);
            drive_fb(0);
        }
        if (tag3Found) {
            drive_lr(0.5);
            sleep(1500);
            drive_lr(0);
            drive_fb(0.5);
            sleep(2500);
            drive_fb(0);
        }
    }
    /**
     * Nach rechts oder nach links fahren.
     *
     * lr = to the left or to the right
     */
    public void drive_lr(double speed_d_lr){
        linksHinten.setPower(speed_d_lr * -1);
        linksVorne.setPower(speed_d_lr);
        rechtsHinten.setPower(speed_d_lr);
        rechtsVorne.setPower(speed_d_lr * -1);
    }

    /**
     * Nach vorne oder nach hinten fahren
     *
     * fb = forward or backward
     */
    public void drive_fb(double speed_d_fb){
        linksHinten.setPower(speed_d_fb);
        linksVorne.setPower(speed_d_fb * -1);
        rechtsHinten.setPower(speed_d_fb);
        rechtsVorne.setPower(speed_d_fb * -1);
    }
}
