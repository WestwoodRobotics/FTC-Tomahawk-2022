//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.openftc.apriltag.AprilTagDetection;
//import org.openftc.easyopencv.OpenCvCamera;
//import org.openftc.easyopencv.OpenCvCameraFactory;
//import org.openftc.easyopencv.OpenCvCameraRotation;
//import org.openftc.easyopencv.OpenCvInternalCamera;
//
//import java.util.ArrayList;
//
//@Autonomous(name="Testoc;eAIton")
//
//public class autoTest extends LinearOpMode {
//    private DcMotorEx frontLeft;
//    private DcMotorEx frontRight;
//    private DcMotorEx backLeft;
//    private DcMotorEx backRight;
//
//    double wheelRadius = 0; // inches
//    int gearRatio = 0; //
//    int trackWidth = 0;
//    double rotPerMat = 24.0/wheelRadius;
//
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
//        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
//        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
//        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
//
//        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
//        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
//        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
//        backRight.setDirection(DcMotorEx.Direction.FORWARD);
//
//        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        frontLeft.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        frontRight.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        backLeft.setVelocityPIDFCoefficients(15, 0, 0, 0);
//        backRight.setVelocityPIDFCoefficients(15, 0, 0, 0);
//
//        waitForStart();
//
//        if (opModeIsActive()){
//
//
//        }
//}
