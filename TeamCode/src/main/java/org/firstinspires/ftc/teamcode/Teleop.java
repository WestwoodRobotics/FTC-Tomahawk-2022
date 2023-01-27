package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import java.util.*;

@TeleOp(name="Tomahawk Teleop")
public class Teleop extends OpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DcMotorEx slide;
    private DcMotorEx slide2;

    private Servo claw1 = null;

    private ElapsedTime runtime = new ElapsedTime();



    //pole heights for slide
    //529.2 is how many ticks per one rotation
    private final int smallPolePos = 487;
    private final int mediumPolePos = 783;
    private final int longPolePos = 1048;

    //18.9 gear ratio - 529 ticksPerRev

    //54.8 gear ratio - 1534 ticksPerRev
    private final int ticksPerRev = 1534;

    // private final int maxRPM = 300;
    // private final int powerVeloCoef = 1500; //ticksPerRev * maxRPM


    @Override

    public void start() {
        runtime.reset();
    }

    @Override
    public void init() {
        telemetry.addData("Status", "Initialized");

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        slide2 = hardwareMap.get(DcMotorEx.class,"slide2");

        claw1 = hardwareMap.get(Servo.class, "claw1");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        slide.setDirection(DcMotorEx.Direction.FORWARD);
        slide2.setDirection(DcMotorSimple.Direction.REVERSE);

        claw1.setDirection(Servo.Direction.REVERSE);

        //Zero Power Behavior
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //ENCODER
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//RESET_ENCODERS **************

    }
    int targetPosition = 0;
    int currentPosition = 0;
    boolean slowMode = false;

    @Override
    public void loop() {
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;
        double slidePower;

        double speedMultiplier;

        if (slowMode) {
            speedMultiplier = 0.2;
        } else {
            speedMultiplier = 0.7;
        }

        //Mecanum Equations
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        frontLeftPower = (drive + strafe + turn) * speedMultiplier;
        frontRightPower = (drive - strafe - turn) * speedMultiplier;
        backLeftPower = (drive - strafe + turn) * speedMultiplier;
        backRightPower = (drive + strafe - turn) * speedMultiplier;

        currentPosition = slide2.getCurrentPosition();

        //Slide Code --------------------------------------------------------------------------------------------
        if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
            if (gamepad2.right_trigger > 0) {
                targetPosition += 3;
            } else if (gamepad2.left_trigger > 0) {
                targetPosition -= 3;
            }
        } else {
            if (gamepad2.a) {
                targetPosition = 0;
            } else if (gamepad2.b) {
                targetPosition = smallPolePos;
            } else if (gamepad2.x) {
                targetPosition = mediumPolePos;
            } else if (gamepad2.y) {
                targetPosition = longPolePos;
            }
        }

        //limits TODO if motors change again
        if (targetPosition > 1100) {
            targetPosition = 1100;
        } else if (targetPosition < 0) {
            targetPosition = 0;
        }
        slide.setTargetPosition(targetPosition);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        slide2.setTargetPosition(targetPosition);
        slide2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (targetPosition > currentPosition) {
//            slide.setPower(.8);
            slide2.setPower(.8);
        } else if (currentPosition > targetPosition) {
//            slide.setPower(-.8);
            slide2.setPower(-.8);
        } else if (currentPosition == targetPosition) {
//            slide.setPower(0);
            slide2.setPower(0);
        }

        //-----------------------------------------------------------------------------------------
        boolean clawOpen = gamepad2.right_bumper;

        if (clawOpen) {
            claw1.setPosition(0.7);

        } else {
            claw1.setPosition(0.2);
        }

        double claw1Pos = claw1.getPosition();

        //Power Setting
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("slide power:", slide.getPower() );

        //fast mode/slow mode toggle
        if (gamepad1.left_bumper) {
            slowMode = false;
        } else if (gamepad1.right_bumper) {
            slowMode = true;
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "SLide Ticks: " + currentPosition);
        telemetry.addData("Status", "Target position: " + targetPosition);
        telemetry.addData("leftTrigger: ", gamepad2.left_trigger);
        telemetry.addData("right trigger: ", gamepad2.right_trigger);
        telemetry.addData("right bumper: ", gamepad2.right_bumper);
        telemetry.addData("Slowmode: ", slowMode);
    }

}