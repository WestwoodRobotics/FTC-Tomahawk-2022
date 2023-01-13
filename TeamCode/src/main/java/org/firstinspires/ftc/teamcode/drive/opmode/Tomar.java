package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Tomar agua")
public class Tomar extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(-38, -69, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        Trajectory df = drive.trajectoryBuilder(startPose)
                .forward(10)
                .build();

        Trajectory toLow = drive.trajectoryBuilder (startPose)
                .splineTo(new Vector2d(-38, -15), Math.toRadians(90))
                .splineTo(new Vector2d(-52, -20), Math.toRadians(270))
//                .strafeRight(56)
                .build();

        waitForStart();

        drive.followTrajectory(toLow);
    }
}
