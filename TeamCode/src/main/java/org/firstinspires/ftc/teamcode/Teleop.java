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

    private ElapsedTime runtime = new ElapsedTime();

    //TODO
//    private Servo claw1;
//    private Servo claw2;

    //pole heights for slide
    //TODO
    //529.2 is how many ticks per one rotation

    private final int smallPolePos = 3726;
    private final int mediumPolePos = 3813;
    private final int longPolePos = 5408;

    private final int ticksPerRev = 529;
    private final int maxRPM = 300;
    private final int powerVeloCoef = ticksPerRev * maxRPM;

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

        //TODO
        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        slide.setDirection(DcMotorEx.Direction.FORWARD);

        //Zero Power Behavior
        frontLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        slide.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //ENCODER
        slide.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        slide.setMode(DcMotor.RunMode.RESET_ENCODERS);
    }
    int targetPositon = 0;
    int currentPosition = 0;

    @Override
    public void loop() {
        double frontLeftPower;
        double frontRightPower;
        double backLeftPower;
        double backRightPower;

        //Mecanum Equations
        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        //UNSURE ABOUT
        frontLeftPower = (drive + strafe + turn) / (1 + Math.sqrt(2.0));
        frontRightPower = (drive - strafe - turn) / (1 + Math.sqrt(2.0));
        backLeftPower = (drive - strafe + turn) / (1 + Math.sqrt(2.0));
        backRightPower = (drive + strafe - turn) / (1 + Math.sqrt(2.0));

        currentPosition = slide.getCurrentPosition();

        //Slide Code
        if (gamepad1.a) {
            slide.setTargetPosition(0);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            targetPositon = 0;
        } else if (gamepad1.b) {
            slide.setTargetPosition(smallPolePos);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            targetPositon = smallPolePos;
        } else if (gamepad1.x) {
            slide.setTargetPosition(mediumPolePos);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            targetPositon = mediumPolePos;
        } else if (gamepad1.y) {
            slide.setTargetPosition(longPolePos);
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            targetPositon = longPolePos;
        }

        if (targetPositon > currentPosition) {
            slide.setPower(.5);
        }
        else if (currentPosition > targetPositon) {
            slide.setPower(-.5);
        }
        else if (currentPosition == targetPositon) {
            slide.setPower(0);
        }

        if (gamepad1.dpad_left) {
            slide.setPower(0);
        }
        if (gamepad1.dpad_up) {
            slide.setPower(.5);
        }
        if (gamepad1.dpad_down) {
            slide.setPower(.5);
        }


        //Power Setting TODO
//        frontLeft.setPower(frontLeftPower);
//        frontRight.setPower(frontRightPower);
//        backLeft.setPower(backLeftPower);
//        backRight.setPower(backRightPower);
        frontLeft.setVelocity(frontLeftPower * powerVeloCoef);
        frontRight.setVelocity(frontRightPower * powerVeloCoef);
        backLeft.setVelocity(backLeftPower * powerVeloCoef);
        backRight.setVelocity(backRightPower * powerVeloCoef);

        telemetry.addData("Status", "Run Time: " + runtime.toString());
    }

}
