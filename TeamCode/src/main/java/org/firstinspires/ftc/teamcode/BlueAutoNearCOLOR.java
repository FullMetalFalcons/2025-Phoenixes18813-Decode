// Copyright Alben Beena 2026 - Version 3.01

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

@Autonomous(name = "Blue Auto Near COLOR", group = "!auto")
public class BlueAutoNearCOLOR extends LinearOpMode {

    DcMotorEx lowerShooter, upperShooter, intake;
    Servo front, mid, back;
    Limelight3A limelight;

    static final double SERVO_DOWN = 1.0;
    static final double SERVO_UP   = 0.0;

    // ========== BATTERY COMPENSATION SETTINGS ==========
    static final double TARGET_VOLTAGE     = 12.31;
    static final double MIN_VOLTAGE        = 11.5;
    static final double MAX_VOLTAGE        = 13.59;
    static final double BASE_SHOOTER_POWER = 0.60;

    static final double INTAKE_POWER = -1.00;

    // ========== DISTANCE-BASED SHOOTER SETTINGS ==========
    static final double CAMERA_HEIGHT_INCHES   = 17.42;
    static final double TARGET_HEIGHT_INCHES   = 29.5;
    static final double CAMERA_ANGLE_DEGREES   = 30.0;

    static final double CLOSE_DISTANCE_INCHES  = 36.0;
    static final double FAR_DISTANCE_INCHES    = 72.0;
    static final double CLOSE_POWER_MULTIPLIER = 0.95;
    static final double FAR_POWER_MULTIPLIER   = 1.05;

    static final double MID_CHAMBER_YAW_OFFSET   = Math.toRadians(0);
    static final double FRONT_CHAMBER_YAW_OFFSET = Math.toRadians(-5);
    static final double BACK_CHAMBER_YAW_OFFSET  = Math.toRadians(2);

    // ========== DECODE APRILTAG IDS ==========
    static final int DECODE_TAG_21 = 21;
    static final int DECODE_TAG_22 = 22;
    static final int DECODE_TAG_23 = 23;

    // ========== DECODE ORDER ==========
    enum BallColor { PURPLE, GREEN, UNKNOWN }
    BallColor[] requiredOrder = new BallColor[3];
    boolean orderDetected = false;
    boolean orderLocked   = false;

    private double currentShooterPower    = BASE_SHOOTER_POWER;
    private double lastCalculatedDistance = 0.0;

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

        lowerShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Start heading -128° so robot faces the DECODE tag board during init scan
        Pose2d startPose = new Pose2d(-50, -50, Math.toRadians(-128));

        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        currentShooterPower = calculateShooterPower(batteryVoltage);

        // ========== SCAN FOR DECODE ORDER DURING INIT ==========
        telemetry.addData("Status", "Scanning for DECODE order...");
        telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
        telemetry.addData("Base Power", "%.2f", currentShooterPower);
        telemetry.addData("Limelight", limelight.isConnected() ? "Connected" : "Disconnected");
        telemetry.update();

        while (!isStarted() && !isStopRequested()) {
            detectDecodeOrder();
            telemetry.addData("Status", "Ready - Waiting for Start");
            telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
            telemetry.addData("Base Shooter Power", "%.2f", currentShooterPower);
            telemetry.addData("DECODE Order", orderLocked ? "LOCKED: " + getOrderString() : (orderDetected ? getOrderString() : "Searching... (point at tag 21/22/23)"));
            telemetry.update();
            sleep(200);
        }

        waitForStart();
        drive.setPoseEstimate(startPose);

        // ========== THE NINE BALL AUTO ==========

        // ========== DEDICATED SCAN TURN ==========
        // Tag board is to the LEFT (negative X) from start.
        // Robot starts at -128°. Turn to face ~180° (left), scan for up to 1s,
        // then turn back to -38° (goal-facing) and drive to shoot.
        // Only runs if order wasn't already locked during init.
        if (!orderLocked) {
            // Turn to face tag board (~180°)
            drive.turn(normalizeAngle(Math.toRadians(180) - Math.toRadians(-128)));

            // Scan repeatedly for up to 1 second
            long scanStart = System.currentTimeMillis();
            while (!orderLocked && System.currentTimeMillis() - scanStart < 1000) {
                detectDecodeOrder();
                telemetry.addData("Scanning DECODE tag", orderLocked ? "LOCKED: " + getOrderString() : "Searching...");
                telemetry.update();
                sleep(50);
            }

            // Turn back to -38° (goal-facing) from wherever we ended up
            drive.turn(normalizeAngle(Math.toRadians(-38) - drive.getPoseEstimate().getHeading()));
        }

        // Travel to shooting position, shooter spins up on the way
        // Drive to (-6,-6) keeping start heading (-128°), shooter spins up on the way
        TrajectorySequence toShootingLine = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .addTemporalMarker(0, () -> setShooterPower(currentShooterPower))
                .lineToConstantHeading(new Vector2d(-6, -6))
                .build();
        drive.followTrajectorySequence(toShootingLine);

        // Turn +40° to face the DECODE tag board and scan for up to 2 seconds
        drive.turn(Math.toRadians(40));
        long scanStart = System.currentTimeMillis();
        while (!orderLocked && System.currentTimeMillis() - scanStart < 2000) {
            detectDecodeOrder();
            telemetry.addData("DECODE Scan", orderLocked ? "LOCKED: " + getOrderString() : "Searching...");
            telemetry.update();
            sleep(50);
        }

        // Turn +47° to face the goal (-128 + 40 + 47 = -41° ≈ shooting heading)
        drive.turn(Math.toRadians(47));

        sleep(200);
        shootBalls(drive); // Shot 1, 2, 3 (in DECODE order if locked, default otherwise)

        // CYCLE 1
        TrajectorySequence cycle1 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(4, -11, Math.toRadians(-90)))
                .addTemporalMarker(0.1, () -> intake.setPower(INTAKE_POWER))
                .lineToConstantHeading(new Vector2d(4, -54))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-6, -6, Math.toRadians(-38)))
                .addTemporalMarker(() -> intake.setPower(0))
                .addTemporalMarker(() -> setShooterPower(currentShooterPower))
                .build();
        drive.followTrajectorySequence(cycle1);

        sleep(150);
        shootBalls(drive); // Shot 4, 5, 6

        // CYCLE 2
        TrajectorySequence cycle2 = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(35, -11, Math.toRadians(-90)))
                .addTemporalMarker(0.1, () -> intake.setPower(INTAKE_POWER))
                .lineToConstantHeading(new Vector2d(35, -65))
                .waitSeconds(0.5)
                .lineToLinearHeading(new Pose2d(-6, -6, Math.toRadians(-38)))
                .addTemporalMarker(() -> intake.setPower(0))
                .addTemporalMarker(() -> setShooterPower(currentShooterPower))
                .build();
        drive.followTrajectorySequence(cycle2);

        sleep(150);
        shootBalls(drive); // Shot 7, 8, 9

        TrajectorySequence returnToPosition = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(17, -17, Math.toRadians(-38)))
                .build();
        drive.followTrajectorySequence(returnToPosition);

        limelight.stop();

        telemetry.addData("Status", "Complete!");
        telemetry.update();
    }

    // ========== DECODE ORDER DETECTION ==========

    void detectDecodeOrder() {
        // Once locked, never overwrite — a momentary loss of sight won't erase the order
        if (orderLocked) return;

        LLResultTypes.FiducialResult tag21 = getTagById(DECODE_TAG_21);
        LLResultTypes.FiducialResult tag22 = getTagById(DECODE_TAG_22);
        LLResultTypes.FiducialResult tag23 = getTagById(DECODE_TAG_23);

        if (tag21 != null) {
            requiredOrder[0] = BallColor.GREEN;
            requiredOrder[1] = BallColor.PURPLE;
            requiredOrder[2] = BallColor.PURPLE;
            orderDetected = true;
            orderLocked   = true;
        } else if (tag22 != null) {
            requiredOrder[0] = BallColor.PURPLE;
            requiredOrder[1] = BallColor.GREEN;
            requiredOrder[2] = BallColor.PURPLE;
            orderDetected = true;
            orderLocked   = true;
        } else if (tag23 != null) {
            requiredOrder[0] = BallColor.PURPLE;
            requiredOrder[1] = BallColor.PURPLE;
            requiredOrder[2] = BallColor.GREEN;
            orderDetected = true;
            orderLocked   = true;
        }
        // No else — if nothing seen, keep whatever we had before
    }

    String getOrderString() {
        if (!orderDetected) return "UNKNOWN";
        return requiredOrder[0] + " -> " + requiredOrder[1] + " -> " + requiredOrder[2];
    }

    // ========== SHOOTING ==========

    void shootBalls(SampleMecanumDrive drive) {
        intake.setPower(INTAKE_POWER);

        telemetry.addData("Base Shooter Power", "%.2f", currentShooterPower);
        telemetry.addData("Shooting Order", orderDetected ? getOrderString() : "Default (mid->front->back)");
        telemetry.update();

        if (orderDetected) {
            for (int i = 0; i < 3; i++) {
                int servoNum = mapColorToServo(requiredOrder[i], i);
                aimAtGoalAndGetHeading(drive, getYawOffsetForServo(servoNum));
                shootBall(servoNum);
            }
        } else {
            aimAtGoalAndGetHeading(drive, MID_CHAMBER_YAW_OFFSET);
            shootBall(2);
            aimAtGoalAndGetHeading(drive, FRONT_CHAMBER_YAW_OFFSET);
            shootBall(1);
            aimAtGoalAndGetHeading(drive, BACK_CHAMBER_YAW_OFFSET);
            shootBall(3);
        }

        intake.setPower(0);
    }

    int mapColorToServo(BallColor color, int positionIndex) {
        if (color == BallColor.GREEN) return 1;
        if (positionIndex == 0 || (positionIndex == 1 && requiredOrder[0] != BallColor.PURPLE)) return 2;
        return 3;
    }

    double getYawOffsetForServo(int servoNum) {
        if (servoNum == 1) return FRONT_CHAMBER_YAW_OFFSET;
        if (servoNum == 2) return MID_CHAMBER_YAW_OFFSET;
        return BACK_CHAMBER_YAW_OFFSET;
    }

    // ========== LIMELIGHT HELPERS ==========

    LLResultTypes.FiducialResult getTagById(int targetId) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == targetId) return f;
        }
        return null;
    }

    LLResultTypes.FiducialResult getGoalTag() {
        return getTagById(20);
    }

    double calculateDistance(LLResultTypes.FiducialResult tag) {
        if (tag == null) return 0.0;
        return Math.abs((TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) /
                Math.tan(Math.toRadians(CAMERA_ANGLE_DEGREES + tag.getTargetYDegrees())));
    }

    double calculateDistanceBasedPower(double distanceInches, double basePower) {
        double powerMultiplier;
        if (distanceInches < CLOSE_DISTANCE_INCHES) {
            powerMultiplier = CLOSE_POWER_MULTIPLIER;
        } else if (distanceInches > FAR_DISTANCE_INCHES) {
            powerMultiplier = FAR_POWER_MULTIPLIER;
        } else {
            double rangeRatio = (distanceInches - CLOSE_DISTANCE_INCHES) /
                    (FAR_DISTANCE_INCHES - CLOSE_DISTANCE_INCHES);
            powerMultiplier = CLOSE_POWER_MULTIPLIER +
                    (rangeRatio * (FAR_POWER_MULTIPLIER - CLOSE_POWER_MULTIPLIER));
        }
        return Math.max(0.6, Math.min(1.0, basePower * powerMultiplier));
    }

    double calculateShooterPower(double voltage) {
        voltage = Math.max(MIN_VOLTAGE, Math.min(MAX_VOLTAGE, voltage));
        return Math.max(0.6, Math.min(1.0, BASE_SHOOTER_POWER * (TARGET_VOLTAGE / voltage)));
    }

    double getYaw(LLResultTypes.FiducialResult tag) {
        if (tag == null) return 0;
        return Math.toRadians(tag.getTargetXDegrees());
    }

    double aimAtGoalAndGetHeading(SampleMecanumDrive drive, double offset) {
        LLResultTypes.FiducialResult tag = getGoalTag();
        if (tag == null) {
            telemetry.addData("Warning", "No AprilTag 20 detected!");
            telemetry.update();
            return drive.getPoseEstimate().getHeading();
        }

        double distance = calculateDistance(tag);
        lastCalculatedDistance = distance;

        double distanceAdjustedPower = calculateDistanceBasedPower(distance, currentShooterPower);
        setShooterPower(distanceAdjustedPower);

        double yawToGoal      = getYaw(tag);
        double currentHeading = drive.getPoseEstimate().getHeading();
        double targetHeading  = currentHeading - yawToGoal + offset;

        turnToHeading(drive, targetHeading);

        telemetry.addData("Distance to Goal", "%.1f in", distance);
        telemetry.addData("Adjusted Power", "%.2f", distanceAdjustedPower);
        telemetry.addData("Yaw Offset", "%.1f°", Math.toDegrees(yawToGoal));
        telemetry.addData("Target Heading", "%.1f°", Math.toDegrees(targetHeading));
        telemetry.update();

        return targetHeading;
    }

    void turnToHeading(SampleMecanumDrive drive, double targetHeading) {
        double currentHeading = drive.getPoseEstimate().getHeading();
        double angleDiff = normalizeAngle(targetHeading - currentHeading);
        if (Math.abs(angleDiff) > Math.toRadians(0.5)) {
            drive.turn(angleDiff);
        }
    }

    double normalizeAngle(double angle) {
        while (angle >  Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    void setShooterPower(double power) {
        lowerShooter.setPower(power);
        upperShooter.setPower(power);
    }

    void stopShooters() {
        lowerShooter.setPower(0);
        upperShooter.setPower(0);
    }

    void shootBall(int n) {
        if (n == 1) {
            front.setPosition(SERVO_UP);
            sleep(650);
            front.setPosition(SERVO_DOWN);
            sleep(150);
        }
        if (n == 2) {
            mid.setPosition(SERVO_UP);
            sleep(675);
            mid.setPosition(SERVO_DOWN);
            sleep(150);
        }
        if (n == 3) {
            back.setPosition(SERVO_UP);
            sleep(650);
            back.setPosition(SERVO_DOWN);
            sleep(150);
        }
    }
}