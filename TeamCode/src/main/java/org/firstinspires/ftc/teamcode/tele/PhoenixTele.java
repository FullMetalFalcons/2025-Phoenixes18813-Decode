package org.firstinspires.ftc.teamcode.tele;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "PhoenixTele", group = "!")
public class PhoenixTele extends LinearOpMode {
    DcMotorEx m1, m2, m3, m4, lift, lift2, arm, extender;
    Servo claw;
    DistanceSensor front, left;

    int liftMax = 0;
    int ExtenderMax = 0;
    int armMax = 0;

    public void runOpMode(){
        m1 = (DcMotorEx)hardwareMap.dcMotor.get("leftFront");
        m2 = (DcMotorEx)hardwareMap.dcMotor.get("rightFront");
        m3 = (DcMotorEx)hardwareMap.dcMotor.get("leftBack");
        m4 = (DcMotorEx)hardwareMap.dcMotor.get("rightBack");

        arm = (DcMotorEx)hardwareMap.dcMotor.get("extender");
        lift = (DcMotorEx)hardwareMap.dcMotor.get("lift");
        lift2 = (DcMotorEx)hardwareMap.dcMotor.get("lift2");

        extender = (DcMotorEx)hardwareMap.dcMotor.get("arm");

        front = hardwareMap.get(DistanceSensor.class, "front_distance");
        left = hardwareMap.get(DistanceSensor.class, "left_distance");

        claw = hardwareMap.servo.get("claw");

        m1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m3.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        m4.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        arm.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        extender.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        lift2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        // m1.setDirection(DcMotorSimple.Direction.REVERSE);
        m2.setDirection(DcMotorSimple.Direction.REVERSE);
        m3.setDirection(DcMotorSimple.Direction.REVERSE);

        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        // lift.setDirection(DcMotorSimple.Direction.REVERSE);
        // lift2.setDirection(DcMotorSimple.Direction.REVERSE);


        arm.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        lift2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        waitForStart();

        while(opModeIsActive()) {
            double px = gamepad1.left_stick_x;
            double py = -gamepad1.left_stick_y;
            double pa = -gamepad1.right_stick_x;

            double p1 = px + py - pa;
            double p2 = -px + py + pa;
            double p3 = -px + py - pa;
            double p4 = px + py + pa;

            double max = Math.max(1.0, Math.abs(p1));
            max = Math.max(max, Math.abs(p2));
            max = Math.max(max, Math.abs(p3));
            max = Math.max(max, Math.abs(p4));

            p1 /= max;
            p2 /= max;
            p3 /= max;
            p4 /= max;

            m1.setPower(p1);
            m2.setPower(p2);
            m3.setPower(p3);
            m4.setPower(p4);

            extender.setPower(-gamepad2.right_stick_y);
            lift.setPower(-gamepad2.left_stick_y);
            lift2.setPower(-gamepad2.left_stick_y);


            arm.setPower(gamepad2.right_trigger-gamepad2.left_trigger);

            if (gamepad1.right_bumper) claw.setPosition(0.68);
            if (gamepad1.left_bumper) claw.setPosition(0.52);

            telemetry.addData("arm", arm.getCurrentPosition());
            telemetry.addData("lift", lift.getCurrentPosition());
            telemetry.addData("lift2", lift2.getCurrentPosition());
            telemetry.addData("claw", claw.getPosition());
            // telemetry.addData("Left Distance", left.getDistance(DistanceUnit.INCH));
            // telemetry.addData("Front Distance", front.getDistance(DistanceUnit.INCH));
            telemetry.update();
        }
    }
}