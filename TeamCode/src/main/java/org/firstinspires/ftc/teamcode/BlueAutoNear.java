//Copyright Alben Beena 2026 - Version 2.09

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

@Autonomous(name = "Blue Auto Near", group = "!auto")
public class BlueAutoNear extends LinearOpMode {

    DcMotorEx lowerShooter, upperShooter, intake;
    Servo front, mid, back;
    Limelight3A limelight;

    static final double SERVO_DOWN = 1.0;
    static final double SERVO_UP   = 0.0;

    // ========== BATTERY COMPENSATION SETTINGS ==========
    static final double TARGET_VOLTAGE = 12.31;
    static final double MIN_VOLTAGE = 11.5;
    static final double MAX_VOLTAGE = 13.59;
    static final double BASE_SHOOTER_POWER = 0.395;

    static final double INTAKE_POWER = -1.00;

    // ========== DISTANCE-BASED SHOOTER SETTINGS ==========

    // Limelight Values
    static final double CAMERA_HEIGHT_INCHES = 17.42;
    static final double TARGET_HEIGHT_INCHES = 29.5;
    static final double CAMERA_ANGLE_DEGREES = 30.0;

    // Shooter power mapping based on distance
    static final double CLOSE_DISTANCE_INCHES = 36.0;
    static final double FAR_DISTANCE_INCHES = 72.0;
    static final double CLOSE_POWER_MULTIPLIER = 0.95;
    static final double FAR_POWER_MULTIPLIER = 1.05;

    static final double MID_CHAMBER_YAW_OFFSET   = Math.toRadians(0);
    static final double FRONT_CHAMBER_YAW_OFFSET = Math.toRadians(-5);
    static final double BACK_CHAMBER_YAW_OFFSET  = Math.toRadians(2);

    private double currentShooterPower = BASE_SHOOTER_POWER;
    private double lastCalculatedDistance = 0.0;

    @Override
    public void runOpMode() throws InterruptedException {

        lowerShooter = hardwareMap.get(DcMotorEx.class, "lowerShooter");
        upperShooter = hardwareMap.get(DcMotorEx.class, "upperShooter");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        front = hardwareMap.get(Servo.class, "front");
        mid   = hardwareMap.get(Servo.class, "mid");
        back  = hardwareMap.get(Servo.class, "back");

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Configure Limelight pipeline
        limelight.pipelineSwitch(0);
        limelight.start();

        front.setPosition(SERVO_DOWN);
        mid.setPosition(SERVO_DOWN);
        back.setPosition(SERVO_DOWN);

        lowerShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        upperShooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-50, -50, Math.toRadians(-38));

        // Calculate battery-compensated shooter power
        double batteryVoltage = hardwareMap.voltageSensor.iterator().next().getVoltage();
        currentShooterPower = calculateShooterPower(batteryVoltage);

        telemetry.addData("Status", "Ready");
        telemetry.addData("Battery Voltage", "%.2f V", batteryVoltage);
        telemetry.addData("Base Power", "%.2f", currentShooterPower);
        telemetry.addData("Target Voltage", "%.2f V", TARGET_VOLTAGE);
        telemetry.addData("Limelight Status", limelight.isConnected() ? "Connected" : "Disconnected");
        telemetry.addData("Camera Height", "%.1f in", CAMERA_HEIGHT_INCHES);
        telemetry.addData("Camera Angle", "%.1f°", CAMERA_ANGLE_DEGREES);
        telemetry.update();

        waitForStart();
        drive.setPoseEstimate(startPose);

        // ========== THE NINE BALL AUTO ==========

        // Start shooter while traveling to first shooting position
        TrajectorySequence toShootingLine = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(0, () -> setShooterPower(currentShooterPower))
                .lineToConstantHeading(new Vector2d(-6, -6))
                .build();
        drive.followTrajectorySequence(toShootingLine);

        sleep(200);
        shootThreeBalls(drive);

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
        shootThreeBalls(drive);

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
        shootThreeBalls(drive);

        TrajectorySequence returnToPosition = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(17, -17, Math.toRadians(-38)))
                .build();
        drive.followTrajectorySequence(returnToPosition);

        limelight.stop();

        telemetry.addData("Status", "Complete!");
        telemetry.update();
    }

    /**
     * Calculate battery-compensated shooter power
     */
    double calculateShooterPower(double voltage) {
        voltage = Math.max(MIN_VOLTAGE, Math.min(MAX_VOLTAGE, voltage));
        double compensatedPower = BASE_SHOOTER_POWER * (TARGET_VOLTAGE / voltage);
        compensatedPower = Math.max(0.6, Math.min(1.0, compensatedPower));
        return compensatedPower;
    }

    /**
     * Calculate distance to AprilTag using Limelight ty (vertical angle)
     * Formula: distance = (targetHeight - cameraHeight) / tan(cameraAngle + ty)
     */
    double calculateDistance(LLResultTypes.FiducialResult tag) {
        if (tag == null) return 0.0;

        double ty = tag.getTargetYDegrees();
        double angleToTarget = CAMERA_ANGLE_DEGREES + ty;

        double heightDifference = TARGET_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES;
        double distance = heightDifference / Math.tan(Math.toRadians(angleToTarget));

        return Math.abs(distance);
    }

    /**
     * Calculate shooter power based on distance and battery voltage
     */
    double calculateDistanceBasedPower(double distanceInches, double basePower) {
        double powerMultiplier = 1.0;

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

        double adjustedPower = basePower * powerMultiplier;
        return Math.max(0.6, Math.min(1.0, adjustedPower));
    }

    void shootThreeBalls(SampleMecanumDrive drive) {

        // Start intake running during the entire shooting sequence
        intake.setPower(INTAKE_POWER);

        telemetry.addData("Base Shooter Power", "%.2f", currentShooterPower);
        telemetry.update();

        // SHOT 1: MID
        aimAtGoalAndGetHeading(drive, MID_CHAMBER_YAW_OFFSET);
        shootBall(2);

        // SHOT 2: FRONT
        aimAtGoalAndGetHeading(drive, FRONT_CHAMBER_YAW_OFFSET);
        shootBall(1);

        // SHOT 3: BACK
        aimAtGoalAndGetHeading(drive, BACK_CHAMBER_YAW_OFFSET);
        shootBall(3);

        // Stop intake after all three balls are fired
        intake.setPower(0);
    }

    /**
     * Gets AprilTag detection with ID 20 from Limelight
     */
    LLResultTypes.FiducialResult getGoalTag() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) {
            return null;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            if (fiducial.getFiducialId() == 20) {
                return fiducial;
            }
        }
        return null;
    }

    /**
     * Calculate yaw (horizontal angle) to the AprilTag
     */
    double getYaw(LLResultTypes.FiducialResult tag) {
        if (tag == null) return 0;
        return Math.toRadians(tag.getTargetXDegrees());
    }

    /**
     * Aim at goal with auto-turn and distance-based power adjustment
     */
    double aimAtGoalAndGetHeading(SampleMecanumDrive drive, double offset) {
        LLResultTypes.FiducialResult tag = getGoalTag();
        if (tag == null) {
            telemetry.addData("Warning", "No AprilTag detected!");
            telemetry.update();
            return drive.getPoseEstimate().getHeading();
        }

        double distance = calculateDistance(tag);
        lastCalculatedDistance = distance;

        double distanceAdjustedPower = calculateDistanceBasedPower(distance, currentShooterPower);
        setShooterPower(distanceAdjustedPower);

        double yawToGoal = getYaw(tag);
        double currentHeading = drive.getPoseEstimate().getHeading();
        double targetHeading = currentHeading - yawToGoal + offset;

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
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    void setShooterPower(double power) {
        lowerShooter.setPower(power);
        upperShooter.setPower(power);
    }

    void stopShooters(){
        lowerShooter.setPower(0);
        upperShooter.setPower(0);
    }

    void shootBall(int n){
        if(n==1){
            front.setPosition(SERVO_UP);
            sleep(650);
            front.setPosition(SERVO_DOWN);
            sleep(150);
        }
        if(n==2){
            mid.setPosition(SERVO_UP);
            sleep(675);
            mid.setPosition(SERVO_DOWN);
            sleep(150);
        }
        if(n==3){
            back.setPosition(SERVO_UP);
            sleep(650);
            back.setPosition(SERVO_DOWN);
            sleep(150);
        }
    }
}