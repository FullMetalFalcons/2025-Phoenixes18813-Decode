//Copyright Alben Beena 2026 - Simple Version

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunnertuning.trajectorysequence.TrajectorySequence;

@Autonomous(name = "Universal Auto Far", group = "!auto")
public class FarAuto extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Starting position
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        telemetry.addData("Status", "Ready - Will move forward 3 feet");
        telemetry.update();

        waitForStart();

        // Move forward 3 feet (36 inches)
        TrajectorySequence moveForward = drive.trajectorySequenceBuilder(startPose)
                .forward(36)
                .build();

        drive.followTrajectorySequence(moveForward);

        telemetry.addData("Status", "Complete!");
        telemetry.update();
    }
}