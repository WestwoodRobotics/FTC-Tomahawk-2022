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

@TeleOp(name="Tomahawk")
public class Teleop extends OpMode {
    private DcMotorEx frontLeft;
    private DcMotorEx frontRight;
    private DcMotorEx backLeft;
    private DcMotorEx backRight;

    private DcMotorEx slide;

    private Servo claw1 = null;

    private ElapsedTime runtime = new ElapsedTime();



    //pole heights for slide
    //TODO
    //529.2 is how many ticks per one rotation

    private final int smallPolePos = 1599;
    private final int mediumPolePos = 3548;
    private final int longPolePos = 3899;

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

        claw1 = hardwareMap.get(Servo.class, "claw1");

        //TODO Directions
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);

        slide.setDirection(DcMotorEx.Direction.FORWARD);

        claw1.setDirection(Servo.Direction.REVERSE);

        //Zero Power Behavior
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //ENCODER
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //RESET_ENCODERS **************

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
            speedMultiplier = 0.3;
        } else {
            speedMultiplier = 0.5;
        }

        //Mecanum Equations
        double drive = gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = -gamepad1.right_stick_x;

        frontLeftPower = (drive + strafe + turn) * speedMultiplier;
        frontRightPower = (drive - strafe - turn) * speedMultiplier;
        backLeftPower = (drive - strafe + turn) * speedMultiplier;
        backRightPower = (drive + strafe - turn) * speedMultiplier;

        currentPosition = slide.getCurrentPosition();

        //Slide Code
        if (gamepad2.left_trigger > 0 || gamepad2.right_trigger > 0) {
//            slide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            if (gamepad2.right_trigger > 0) {
//                slide.setPower(.8);
                targetPosition += 20;
            } else if (gamepad2.left_trigger > 0) {
//                slide.setPower(-.8);
                targetPosition -= 20;
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

        //limits
        if (targetPosition > 5000) {
            targetPosition = 5000;
        } else if (targetPosition < 0) {
            targetPosition = 0;
        }
        slide.setTargetPosition(targetPosition);
        slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (targetPosition > currentPosition) {
            slide.setPower(.8);
        } else if (currentPosition > targetPosition) {
            slide.setPower(-.8);
        } else if (currentPosition == targetPosition) {
            slide.setPower(0);
        }

        boolean clawOpen = gamepad2.right_bumper;

        if (clawOpen) {
            claw1.setPosition(0);

        } else {
            claw1.setPosition(1);
        }

        double claw1Pos = claw1.getPosition();

        //Power Setting TODO
        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("slide power:", slide.getPower() );

        //fast mode/slow mode toggle
        if (gamepad1.dpad_up) {
            slowMode = false;
        } else if (gamepad1.dpad_down) {
            slowMode = true;
        }

        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Status", "SLide Ticks: " + currentPosition);
        telemetry.addData("leftTrigger: ", gamepad2.left_trigger);
        telemetry.addData("right trigger: ", gamepad2.right_trigger);
    }

}