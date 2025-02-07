package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;

@Config
@TeleOp(group = "Tests")
public class outreachBot extends LinearOpMode {
    Follower follower;
    Drivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            hardware.motors.leftFront.initMotor(hardwareMap);
            hardware.motors.rightBack.initMotor(hardwareMap);
            double fwd = -gamepad1.left_stick_y;
            double rot = gamepad1.right_stick_x;
            hardware.motors.leftFront.setPower(fwd - rot);
            hardware.motors.rightBack.setPower(fwd + rot);
//            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
//            follower.update();
        }
    }
}