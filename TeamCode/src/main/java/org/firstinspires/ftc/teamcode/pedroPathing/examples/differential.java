package org.firstinspires.ftc.teamcode.pedroPathing.examples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.DifferentialSwerveDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.DifferentialSwervePod;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;

@Config
@TeleOp(group = "Tests")
public class differential extends LinearOpMode {
    double[] motorPowers = new double[4];

    DifferentialSwervePod leftPod, rightPod;

    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        leftPod = new DifferentialSwervePod(new Point(0,10,Point.CARTESIAN));

        rightPod = new DifferentialSwervePod(new Point(0,-10,Point.CARTESIAN));

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
//            motorPowers = drivetrain.getDrivePowers(new Vector(0,0), new Vector(rotate,0), input, 0);
//            for (int i = 0; i < drivetrain.drivetrainMotors.size(); i++) {
//                drivetrain.drivetrainMotors.get(i).setPower(motorPowers[i]);
//            }
            drivetrain.run(new Vector(0,0), new Vector(rotate,0), input, 0);


            telemetry.addData("input r", input.getMagnitude());
            telemetry.addData("input theta", Math.toDegrees(input.getTheta()));
            telemetry.update();
        }
    }
}