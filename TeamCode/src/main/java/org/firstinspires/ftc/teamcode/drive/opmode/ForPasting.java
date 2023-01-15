//.splineTo(new Vector2d(-22, -25), Math.toRadians(65))
//
//
//@@ -0,0 +1,341 @@
//        package org.firstinspires.ftc.teamcode.HM2022;
//        /*
//         * Copyright (c) 2021 OpenFTC Team
//         *
//         * Permission is hereby granted, free of charge, to any person obtaining a copy
//         * of this software and associated documentation files (the "Software"), to deal
//         * in the Software without restriction, including without limitation the rights
//         * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//         * copies of the Software, and to permit persons to whom the Software is
//         * furnished to do so, subject to the following conditions:
//         *
//         * The above copyright notice and this permission notice shall be included in all
//         * copies or substantial portions of the Software.
//         * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//         * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//         * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//         * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//         * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//         * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//         * SOFTWARE.
//         */
//        import com.acmerobotics.roadrunner.geometry.Pose2d;
//        import com.acmerobotics.roadrunner.geometry.Vector2d;
//        import com.acmerobotics.roadrunner.trajectory.Trajectory;
//        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//        import com.qualcomm.robotcore.hardware.DcMotor;
//        import com.qualcomm.robotcore.hardware.Servo;
//        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//        import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
//        import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//        import org.openftc.apriltag.AprilTagDetection;
//        import org.openftc.easyopencv.OpenCvCamera;
//        import org.openftc.easyopencv.OpenCvCameraFactory;
//        import org.openftc.easyopencv.OpenCvCameraRotation;
//        import java.util.ArrayList;
//@Autonomous(name = "LowAttempt")
//public class LowAttempt extends LinearOpMode
//{
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//    static final double FEET_PER_METER = 3.28084;
//    String parking = "middle";
//    Servo claw = null;
//    DcMotor viperSlide = null;
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
//    int LEFT = 1;
//    int MIDDLE = 2;
//    int RIGHT = 3;
//    AprilTagDetection tagOfInterest = null;
//    @Override
//    public void runOpMode()
//    {
////        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
////        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
////        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//        claw = hardwareMap.get(Servo.class, "claw");
//        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
//        viperSlide.setDirection(DcMotor.Direction.FORWARD);
//        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        viperSlide.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        camera.setPipeline(aprilTagDetectionPipeline);
////        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
////        {
////            @Override
////            public void onOpened()
////            {
////                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
////            }
////            @Override
////            public void onError(int errorCode)
////            {
////            }
////        });
//        telemetry.setMsTransmissionInterval(50);
//        /*
//         * The INIT-loop:
//         * This REPLACES waitForStart!
//         */
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d startPose = new Pose2d(-36, -68, Math.toRadians(90));
//
//        Trajectory toLow = drive.trajectoryBuilder (startPose)
//                .splineTo(new Vector2d(-49, -24), Math.toRadians(200))
//                .build();
//
//        Trajectory toStack01 = drive.trajectoryBuilder(new Pose2d(-49, -24), Math.toRadians(175))
//                .back(9)
//                .build();
//
//        Trajectory toStack11 = drive.trajectoryBuilder(toStack01.end())
//                .strafeRight(18)
//                .build();
//
//        Trajectory toStack21 = drive. trajectoryBuilder(toStack11.end())
//                .splineTo(new Vector2d( -66, -14), Math.toRadians(177))
//                .build();
//
//        Trajectory backLow1 = drive.trajectoryBuilder(new Pose2d(-66, -14), Math.toRadians(173))
//                .splineTo(new Vector2d(-51, -28), Math.toRadians(270))
//                .build();
//
//        Trajectory backLow2 = drive.trajectoryBuilder(backLow1.end())
//                .forward(6)
//                .build();
//        Trajectory backBackLow = drive.trajectoryBuilder(backLow2.end())
//                .forward()
//                .build();
//
////        Trajectory middle = drive.trajectoryBuilder(toLowFR2.end())
////                .splineTo(new Vector2d(40, 36), Math.toRadians(90))
////                .build();
////
////        Trajectory left = drive.trajectoryBuilder(middle.end())
////                .strafeLeft(22)
////                .build();
////        Trajectory right = drive.trajectoryBuilder(middle.end())
////                .strafeRight(25)
////                .build();
//        while (!isStarted() && !isStopRequested())
//        {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//                if(tagFound)
//                {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("Don't see tag of interest :(");
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//            }
//            telemetry.update();
//            sleep(20);
//        }
//        /*
//         * The START command just came in: now work off the latest snapshot acquired
//         * during the init loop.
//         */
//        /* Update the telemetry */
//        if(tagOfInterest != null)
//        {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }
//        /* Actually do something useful */
//        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
//            parking = "Left";
//        } else if (tagOfInterest.id == MIDDLE) {
//            parking = "Middle";
//        } else {
//            parking = "Right";
//        }
//
//        /* picking up pre-placed cone */
//        claw.setPosition(1);
//        claw.setPosition(0.69);
//        sleep(800);
//
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(1100);
//        viperSlide.setPower(0);
//        drive.followTrajectory(toLow);
//
//        claw.setPosition(0.4);
//        sleep(400);
//
//        drive.followTrajectory(toStack01);
//        drive.followTrajectory(toStack11);
//        drive.followTrajectory(toStack21);
//        // 5th cone
//        viperSlide.setTargetPosition(630);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(-1);
//        sleep(800);
//        viperSlide.setPower(0);
//        claw.setPosition(1);
//        sleep(600);
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(800);
//        viperSlide.setPower(0);
//        drive.followTrajectory(backLow1);
//        drive.followTrajectory(backLow2);
//        claw.setPosition(0.4);
//        sleep(400);
//        drive.followTrajectory(toStack01);
//        drive.followTrajectory(toStack11);
//        drive.followTrajectory(toStack21);
//        // 4th cone
//        viperSlide.setTargetPosition(440);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(-1);
//        sleep(850);
//        viperSlide.setPower(0);
//        claw.setPosition(1);
//        sleep(600);
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(850);
//        viperSlide.setPower(0);
//        drive.followTrajectory(backLow1);
//        drive.followTrajectory(backLow2);
//        claw.setPosition(0.4);
//        sleep(400);
//        drive.followTrajectory(toStack01);
//        drive.followTrajectory(toStack11);
//        drive.followTrajectory(toStack21);
//        // 3rd cone
//        viperSlide.setTargetPosition(320);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(-1);
//        sleep(1000);
//        viperSlide.setPower(0);
//        claw.setPosition(1);
//        sleep(600);
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(1000);
//        viperSlide.setPower(0);
//        drive.followTrajectory(backLow1);
//        drive.followTrajectory(backLow2);
//        claw.setPosition(0.4);
//        sleep(400);
//        drive.followTrajectory(toStack01);
//        drive.followTrajectory(toStack11);
//        drive.followTrajectory(toStack21);
//        // 2nd cone
//        viperSlide.setTargetPosition(190);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(-1);
//        sleep(1100);
//        viperSlide.setPower(0);
//        claw.setPosition(1);
//        sleep(600);
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(1100);
//        viperSlide.setPower(0);
//        drive.followTrajectory(backLow1);
//        drive.followTrajectory(backLow2);
//        claw.setPosition(0.4);
//        sleep(400);
//        drive.followTrajectory(toStack01);
//        drive.followTrajectory(toStack11);
//        drive.followTrajectory(toStack21);
//        // 1st cone
//        viperSlide.setTargetPosition(10);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(-1);
//        sleep(1250);
//        viperSlide.setPower(0);
//        claw.setPosition(1);
//        sleep(500);
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(1250);
//        viperSlide.setPower(0);
//        drive.followTrajectory(backLow1);
//        drive.followTrajectory(backLow2);
//        claw.setPosition(0.4);
//        sleep(400);
////        drive.followTrajectory(middle);
////        if (parking == "Left") {
////            drive.followTrajectory(left);
////        } else if (parking == "Right") {
////            drive.followTrajectory(right);
////        }
//        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//        while (opModeIsActive()) {sleep(20);}
//    }
//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
//}
//
//
//
//
//
//
//
//
//
//
//package org.firstinspires.ftc.teamcode.HM2022;
//        /*
//         * Copyright (c) 2021 OpenFTC Team
//         *
//         * Permission is hereby granted, free of charge, to any person obtaining a copy
//         * of this software and associated documentation files (the "Software"), to deal
//         * in the Software without restriction, including without limitation the rights
//         * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//         * copies of the Software, and to permit persons to whom the Software is
//         * furnished to do so, subject to the following conditions:
//         *
//         * The above copyright notice and this permission notice shall be included in all
//         * copies or substantial portions of the Software.
//         * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//         * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//         * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//         * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//         * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//         * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//         * SOFTWARE.
//         */
//        import com.acmerobotics.roadrunner.geometry.Pose2d;
//        import com.acmerobotics.roadrunner.geometry.Vector2d;
//        import com.acmerobotics.roadrunner.trajectory.Trajectory;
//        import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//        import com.qualcomm.robotcore.hardware.DcMotor;
//        import com.qualcomm.robotcore.hardware.Servo;
//        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//        import org.firstinspires.ftc.teamcode.AprilTagDetectionPipeline;
//        import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
//        import org.openftc.apriltag.AprilTagDetection;
//        import org.openftc.easyopencv.OpenCvCamera;
//        import org.openftc.easyopencv.OpenCvCameraFactory;
//        import org.openftc.easyopencv.OpenCvCameraRotation;
//        import java.util.ArrayList;
//@Autonomous(name = "Low")
//public class Low extends LinearOpMode
//{
//    OpenCvCamera camera;
//    AprilTagDetectionPipeline aprilTagDetectionPipeline;
//    static final double FEET_PER_METER = 3.28084;
//    String parking = "middle";
//    Servo claw = null;
//    DcMotor viperSlide = null;
//    // Lens intrinsics
//    // UNITS ARE PIXELS
//    // NOTE: this calibration is for the C920 webcam at 800x448.
//    // You will need to do your own calibration for other configurations!
//    double fx = 578.272;
//    double fy = 578.272;
//    double cx = 402.145;
//    double cy = 221.506;
//    // UNITS ARE METERS
//    double tagsize = 0.166;
//    //    int ID_TAG_OF_INTEREST = 18; // Tag ID 18 from the 36h11 family
//    int LEFT = 1;
//    int MIDDLE = 2;
//    int RIGHT = 3;
//    AprilTagDetection tagOfInterest = null;
//    @Override
//    public void runOpMode()
//    {
////        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
////        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
////        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
//        claw = hardwareMap.get(Servo.class, "claw");
//        viperSlide = hardwareMap.get(DcMotor.class, "viperSlide");
//        viperSlide.setDirection(DcMotor.Direction.FORWARD);
//        viperSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        viperSlide.setMode(DcMotor.RunMode.RESET_ENCODERS);
//        viperSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
////        camera.setPipeline(aprilTagDetectionPipeline);
////        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
////        {
////            @Override
////            public void onOpened()
////            {
////                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
////            }
////            @Override
////            public void onError(int errorCode)
////            {
////            }
////        });
//        telemetry.setMsTransmissionInterval(50);
//        /*
//         * The INIT-loop:
//         * This REPLACES waitForStart!
//         */
//        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
//
//        Pose2d startPose = new Pose2d(-36, -68, Math.toRadians(90));
//
//        Trajectory toLow = drive.trajectoryBuilder (startPose)
//                .splineTo(new Vector2d(-36, -35), Math.toRadians(90))
//                .splineTo(new Vector2d(-50, -28), Math.toRadians(280))
//                .build();
//
//        Trajectory toStack01 = drive.trajectoryBuilder(new Pose2d(-49, -28), Math.toRadians(270))
//                .back(18)
//                .build();
//
//        Trajectory toStack21 = drive. trajectoryBuilder(toStack01.end())
//                .splineTo(new Vector2d( -66, -14), Math.toRadians(177))
//                .build();
//
//        Trajectory backLow1 = drive.trajectoryBuilder(new Pose2d(-66, -14), Math.toRadians(173), true)
//                .splineTo(new Vector2d(-51, -18), Math.toRadians(270))
//                .build();
//
//        Trajectory backLow2 = drive.trajectoryBuilder(backLow1.end())
//                .forward(11)
//                .build();
//
//        Trajectory middle = drive.trajectoryBuilder(backLow2.end(), true)
//                .splineTo(new Vector2d(-40, -36), Math.toRadians(90))
//                .build();
//
//        Trajectory left = drive.trajectoryBuilder(middle.end())
//                .strafeLeft(22)
//                .build();
//
//        Trajectory right = drive.trajectoryBuilder(middle.end())
//                .strafeRight(25)
//                .build();
//        while (!isStarted() && !isStopRequested())
//        {
//            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
//            if(currentDetections.size() != 0)
//            {
//                boolean tagFound = false;
//                for(AprilTagDetection tag : currentDetections)
//                {
//                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
//                    {
//                        tagOfInterest = tag;
//                        tagFound = true;
//                        break;
//                    }
//                }
//                if(tagFound)
//                {
//                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
//                    tagToTelemetry(tagOfInterest);
//                }
//                else
//                {
//                    telemetry.addLine("Don't see tag of interest :(");
//                    if(tagOfInterest == null)
//                    {
//                        telemetry.addLine("(The tag has never been seen)");
//                    }
//                    else
//                    {
//                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                        tagToTelemetry(tagOfInterest);
//                    }
//                }
//            }
//            else
//            {
//                telemetry.addLine("Don't see tag of interest :(");
//                if(tagOfInterest == null)
//                {
//                    telemetry.addLine("(The tag has never been seen)");
//                }
//                else
//                {
//                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
//                    tagToTelemetry(tagOfInterest);
//                }
//            }
//            telemetry.update();
//            sleep(20);
//        }
//        /*
//         * The START command just came in: now work off the latest snapshot acquired
//         * during the init loop.
//         */
//        /* Update the telemetry */
//        if(tagOfInterest != null)
//        {
//            telemetry.addLine("Tag snapshot:\n");
//            tagToTelemetry(tagOfInterest);
//            telemetry.update();
//        }
//        else
//        {
//            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
//            telemetry.update();
//        }
//        /* Actually do something useful */
//        if(tagOfInterest == null || tagOfInterest.id == LEFT) {
//            parking = "Left";
//        } else if (tagOfInterest.id == MIDDLE) {
//            parking = "Middle";
//        } else {
//            parking = "Right";
//        }
//
//        /* picking up pre-placed cone */
//        claw.setPosition(1);
//        sleep(600);
//
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(900);
//        viperSlide.setPower(0);
//        drive.followTrajectory(toLow);
//
//        claw.setPosition(0.4);
//        sleep(400);
//
//        drive.followTrajectory(toStack01);
//        drive.followTrajectory(toStack21);
//        // 5th cone
//        viperSlide.setTargetPosition(630);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(-1);
//        sleep(800);
//        viperSlide.setPower(0);
//        claw.setPosition(1);
//        sleep(600);
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(800);
//        viperSlide.setPower(0);
//        drive.followTrajectory(backLow1);
//        drive.followTrajectory(backLow2);
//        claw.setPosition(0.4);
//        sleep(400);
//        drive.followTrajectory(toStack01);
//        drive.followTrajectory(toStack21);
//        // 4th cone
//        viperSlide.setTargetPosition(440);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(-1);
//        sleep(850);
//        viperSlide.setPower(0);
//        claw.setPosition(1);
//        sleep(600);
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(850);
//        viperSlide.setPower(0);
//        drive.followTrajectory(backLow1);
//        drive.followTrajectory(backLow2);
//        claw.setPosition(0.4);
//        sleep(400);
//        drive.followTrajectory(toStack01);
//        drive.followTrajectory(toStack21);
//        // 3rd cone
//        viperSlide.setTargetPosition(320);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(-1);
//        sleep(1000);
//        viperSlide.setPower(0);
//        claw.setPosition(1);
//        sleep(600);
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(1000);
//        viperSlide.setPower(0);
//        drive.followTrajectory(backLow1);
//        drive.followTrajectory(backLow2);
//        claw.setPosition(0.4);
//        sleep(400);
//        drive.followTrajectory(toStack01);
//        drive.followTrajectory(toStack21);
//        // 2nd cone
//        viperSlide.setTargetPosition(190);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(-1);
//        sleep(1100);
//        viperSlide.setPower(0);
//        claw.setPosition(1);
//        sleep(600);
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(1100);
//        viperSlide.setPower(0);
//        drive.followTrajectory(backLow1);
//        drive.followTrajectory(backLow2);
//        claw.setPosition(0.4);
//        sleep(400);
//        drive.followTrajectory(toStack01);
//        drive.followTrajectory(toStack21);
//        // 1st cone
//        viperSlide.setTargetPosition(10);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(-1);
//        sleep(1250);
//        viperSlide.setPower(0);
//        claw.setPosition(1);
//        sleep(500);
//        viperSlide.setTargetPosition(1400);
//        viperSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        viperSlide.setPower(1);
//        sleep(1250);
//        viperSlide.setPower(0);
//        drive.followTrajectory(backLow1);
//        drive.followTrajectory(backLow2);
//        claw.setPosition(0.4);
//        sleep(400);
//        drive.followTrajectory(middle);
////        if (parking == "Left") {
////            drive.followTrajectory(left);
////        } else if (parking == "Right") {
////            drive.followTrajectory(right);
////        }
//        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
//        while (opModeIsActive()) {sleep(20);}
//    }
//    void tagToTelemetry(AprilTagDetection detection)
//    {
//        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
//        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
//        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
//        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
//        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
//        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
//    }
//}
//
