package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.DifferentialSwerveDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.DifferentialSwervePod;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

import java.util.Arrays;
import java.util.List;

@Config
@TeleOp(group = "Tests")
public class differential extends LinearOpMode {
    protected List<DcMotorEx> motors;

    double[] motorPowers = new double[4];

    DifferentialSwervePod leftPod, rightPod;

    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx leftFrontTop = hardwareMap.get(DcMotorEx.class, "blue_front");
        DcMotorEx leftFrontBottom = hardwareMap.get(DcMotorEx.class, "blue_back");
        DcMotorEx rightFrontTop = hardwareMap.get(DcMotorEx.class, "red_front");
        DcMotorEx rightFrontBottom = hardwareMap.get(DcMotorEx.class, "red_back");
        leftFrontTop.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontBottom.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFrontTop.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFrontBottom.setDirection(DcMotorSimple.Direction.FORWARD);

        motors = Arrays.asList(leftFrontTop, leftFrontBottom, rightFrontTop, rightFrontBottom);
        motors.get(1).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(1).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motors.get(3).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motors.get(3).setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        leftPod = new DifferentialSwervePod(
                new Point(0,10,Point.CARTESIAN),
                motors.get(1));

        rightPod = new DifferentialSwervePod(
                new Point(0,-10,Point.CARTESIAN),
                motors.get(3));

        drivetrain = new DifferentialSwerveDrivetrain(hardwareMap, leftPod, rightPod);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            Vector input = new Vector(new Point (-gamepad1.left_stick_y, -gamepad1.left_stick_x));
            double rotate = -0.5 * gamepad1.right_stick_x;
//            double[] valuesLeft = leftPod.calculateMotorPowers(input, rotate);
//            motors.get(0).setPower(valuesLeft[0]);
//            motors.get(1).setPower(valuesLeft[1]);
//
//            double[] valuesRight = rightPod.calculateMotorPowers(input, rotate);
//            motors.get(2).setPower(valuesRight[0]);
//            motors.get(3).setPower(valuesRight[1]);
            motorPowers = drivetrain.getDrivePowers(new Vector(0,0), new Vector(rotate,0), input, 0);
            for (int i = 0; i < motors.size(); i++) {
                motors.get(i).setPower(motorPowers[i]);
            }
            //FIXME
//            drivetrain.run(new Vector(0,0), new Vector(rotate,0), input, 0);


            telemetry.addData("input r", input.getMagnitude());
            telemetry.addData("input theta", Math.toDegrees(input.getTheta()));
            telemetry.update();
        }
    }
}