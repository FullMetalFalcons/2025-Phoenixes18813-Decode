// Copyright Alben Beena 2026 - Version 3.1 - RED MIRRORED + 12 BALL OPTIMIZED
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.roadrunnertuning.drive.SampleMecanumDrive;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.roadrunnertuning.trajectorysequence.TrajectorySequence;
import com.qualcomm.robotcore.hardware.*;

import java.util.List;

@Autonomous(name = "Red Auto Near V7 - Optimized", group = "!auto")
public class RedAutoNearV5 extends LinearOpMode {

    // ─── Hardware ───────────────────────────────────────────────────────────────
    DcMotorEx lowerShooter, upperShooter, intake;
    Servo front, mid, back;
    Limelight3A limelight;
    SampleMecanumDrive drive;

    // ─── Servo Positions ────────────────────────────────────────────────────────
    static final double SERVO_DOWN = 1.0;
    static final double SERVO_UP   = 0.0;

    // ─── Velocity Settings (From Blue V7) ───────────────────────────────────────
    static final double MAX_VELOCITY_TICKS    = 2800.0;
    static final double BASE_SHOOTER_VELOCITY = 395.0;
    static final double SIMUL_VEL_BOOST       = 1.12;
    static final double INTAKE_POWER          = -1.00;

    // ─── Limelight / Distance Settings ──────────────────────────────────────────
    static final double CAMERA_HEIGHT_INCHES  = 17.42;
    static final double TARGET_HEIGHT_INCHES  = 29.5;
    static final double CAMERA_ANGLE_DEGREES  = 30.0;
    static final double CLOSE_DISTANCE_INCHES = 36.0;
    static final double FAR_DISTANCE_INCHES   = 72.0;
    static final double CLOSE_VEL_MULTIPLIER  = 0.95;
    static final double FAR_VEL_MULTIPLIER    = 1.05;

    static final double SIMUL_YAW_OFFSET = Math.toRadians(0.75);

    // ─── State ──────────────────────────────────────────────────────────────────
    private double currentTargetVelocity  = BASE_SHOOTER_VELOCITY;
    private double lastCalculatedDistance = 48.0;

    @Override
    public void runOpMode() throws InterruptedException {

        lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");
        upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
        intake       = hardwareMap.get(DcMotorEx.class, "intake");

        front = hardwareMap.get(Servo.class, "front");
        mid   = hardwareMap.get(Servo.class, "mid");
        back  = hardwareMap.get(Servo.class, "back");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        front.setPosition(SERVO_DOWN);
        mid.setPosition(SERVO_DOWN);
        back.setPosition(SERVO_DOWN);

        // Switching to Encoder Velocity mode for V7 consistency
        lowerShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upperShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lowerShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upperShooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        drive = new SampleMecanumDrive(hardwareMap);
        // Kept your Red Start Pose
        Pose2d startPose = new Pose2d(-50, 50, Math.toRadians(218));

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("═══ RED OPTIMIZED V7 ═══");
            telemetry.addData("Base Velocity", BASE_SHOOTER_VELOCITY);
            telemetry.update();
        }

        waitForStart();
        drive.setPoseEstimate(startPose);

        // TIME SAVE: Spin up shooters AND run intake from the start
        applyShooterVelocity(BASE_SHOOTER_VELOCITY);
        intake.setPower(INTAKE_POWER);

        // ── CYCLE 0 (Preloads) ──────────────────────────────────────────────
        TrajectorySequence toShootingLine = drive.trajectorySequenceBuilder(startPose)
                .lineToConstantHeading(new Vector2d(-8, 8))
                .build();
        drive.followTrajectorySequence(toShootingLine);
        shootAllThreeSimultaneous();

        // ── CYCLE 1 ──────────────────────────────────────────────────────────
        TrajectorySequence cycle1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(4.75, 9.5, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(4.75, 53))
                .addTemporalMarker(() -> applyShooterVelocity(BASE_SHOOTER_VELOCITY))
                .lineToLinearHeading(new Pose2d(-5.75, 5.75, Math.toRadians(250)))
                .build();
        drive.followTrajectorySequence(cycle1);
        shootAllThreeSimultaneous();

        // ── CYCLE 2 ──────────────────────────────────────────────────────────
        TrajectorySequence cycle2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(34.9, 9.5, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(34.9, 63))
                .addTemporalMarker(() -> applyShooterVelocity(BASE_SHOOTER_VELOCITY))
                .lineToLinearHeading(new Pose2d(-5.75, 5.75, Math.toRadians(250)))
                .build();
        drive.followTrajectorySequence(cycle2);
        shootAllThreeSimultaneous();

        // ── CYCLE 3 (Added for 12 Balls) ─────────────────────────────────────
        TrajectorySequence cycle3 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(64, 11, Math.toRadians(90)))
                .lineToConstantHeading(new Vector2d(64, 63))
                .addTemporalMarker(() -> applyShooterVelocity(BASE_SHOOTER_VELOCITY))
                .lineToLinearHeading(new Pose2d(-5.75, 5.75, Math.toRadians(250)))
                .build();
        drive.followTrajectorySequence(cycle3);
        shootAllThreeSimultaneous();

        intake.setPower(0);
        limelight.stop();
        stopShooters();
    }

    // ─── Core Shooting Logic (From Blue V7) ─────────────────────────────────────

    void shootAllThreeSimultaneous() {
        intake.setPower(0);
        aimAtGoalAndGetHeading(SIMUL_YAW_OFFSET);

        double boostedVel = Math.min(
                MAX_VELOCITY_TICKS,
                calculateDistanceBasedVelocity(lastCalculatedDistance) * SIMUL_VEL_BOOST
        );
        applyShooterVelocity(boostedVel);
        waitUntilVelocityStable(boostedVel);

        front.setPosition(SERVO_UP);
        mid.setPosition(SERVO_UP);
        sleep(330);
        back.setPosition(SERVO_UP);
        sleep(550);

        front.setPosition(SERVO_DOWN);
        mid.setPosition(SERVO_DOWN);
        back.setPosition(SERVO_DOWN);
        sleep(100);

        intake.setPower(INTAKE_POWER);
    }

    void aimAtGoalAndGetHeading(double offset) {
        LLResultTypes.FiducialResult tag = getGoalTag();
        double distance = (tag != null) ? calculateDistance(tag) : lastCalculatedDistance;
        applyShooterVelocity(calculateDistanceBasedVelocity(distance));

        if (tag != null) {
            double yawToGoal     = Math.toRadians(tag.getTargetXDegrees());
            double targetHeading = drive.getPoseEstimate().getHeading() - yawToGoal + offset;
            turnToHeading(targetHeading);
        }
    }

    void turnToHeading(double targetHeading) {
        double angleDiff = normalizeAngle(targetHeading - drive.getPoseEstimate().getHeading());
        if (Math.abs(angleDiff) > Math.toRadians(0.5)) drive.turn(angleDiff);
    }

    double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    void applyShooterVelocity(double velocity) {
        currentTargetVelocity = velocity;
        lowerShooter.setVelocity(velocity);
        upperShooter.setVelocity(velocity);
    }

    void waitUntilVelocityStable(double target) {
        long startTime = System.currentTimeMillis();
        while (opModeIsActive() && (System.currentTimeMillis() - startTime < 400)) {
            double current = lowerShooter.getVelocity();
            if (Math.abs(current - target) < (target * 0.03)) break;
        }
    }

    void stopShooters() {
        lowerShooter.setVelocity(0);
        upperShooter.setVelocity(0);
    }

    // ─── Math & Detection ──────────────────────────────────────────────────────

    double calculateDistance(LLResultTypes.FiducialResult tag) {
        if (tag == null) return lastCalculatedDistance;
        double ty            = tag.getTargetYDegrees();
        double angleToTarget = CAMERA_ANGLE_DEGREES + ty;
        double heightDiff    = TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;
        double distance      = heightDiff / Math.tan(Math.toRadians(angleToTarget));
        lastCalculatedDistance = Math.abs(distance);
        return lastCalculatedDistance;
    }

    double calculateDistanceBasedVelocity(double distanceInches) {
        double multiplier;
        if (distanceInches < CLOSE_DISTANCE_INCHES) {
            multiplier = CLOSE_VEL_MULTIPLIER;
        } else if (distanceInches > FAR_DISTANCE_INCHES) {
            multiplier = FAR_VEL_MULTIPLIER;
        } else {
            double rangeRatio = (distanceInches - CLOSE_DISTANCE_INCHES)
                    / (FAR_DISTANCE_INCHES - CLOSE_DISTANCE_INCHES);
            multiplier = CLOSE_VEL_MULTIPLIER + (rangeRatio * (FAR_VEL_MULTIPLIER - CLOSE_VEL_MULTIPLIER));
        }
        return BASE_SHOOTER_VELOCITY * multiplier;
    }

    LLResultTypes.FiducialResult getGoalTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null) return null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == 24) return f; // Tag 24 = Red Goal
        }
        return null;
    }
}