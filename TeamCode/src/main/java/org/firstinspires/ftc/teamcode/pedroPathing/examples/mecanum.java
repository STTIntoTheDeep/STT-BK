package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.DifferentialSwerveDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.DifferentialSwervePod;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

@Config
@TeleOp(group = "Tests")
public class mecanum extends LinearOpMode {
    Follower follower;
    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = new Follower(hardwareMap);
        follower.startTeleopDrive();
        drivetrain = new MecanumDrivetrain(hardwareMap);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
//            hardware.motors.leftFront.setPower(-gamepad1.left_stick_y);
//            hardware.motors.leftBack.setPower(-gamepad1.right_stick_x);
//            hardware.motors.rightFront.setPower(gamepad1.right_trigger);
//            hardware.motors.rightBack.setPower(gamepad1.left_trigger);
//            Vector input = new Vector(new Point(-gamepad1.left_stick_y, -gamepad1.left_stick_x, Point.CARTESIAN));
//            drivetrain.run(new Vector(0,0), new Vector (-gamepad1.right_stick_x,0), input,  0);

            telemetry.addData("y", -gamepad1.left_stick_y);
            telemetry.addData("x", -gamepad1.left_stick_x);
            telemetry.addData("lf",hardware.motors.leftFront.getLastPower());
            telemetry.addData("lb",hardware.motors.leftBack.getLastPower());
            telemetry.addData("rf",hardware.motors.rightFront.getLastPower());
            telemetry.addData("rb",hardware.motors.rightBack.getLastPower());
//            telemetry.addData("r",input.getMagnitude());
//            telemetry.addData("theta",Math.toDegrees(input.getTheta()));
            telemetry.update();
            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
        }
    }
}