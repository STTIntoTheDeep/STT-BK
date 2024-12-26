package org.firstinspires.ftc.teamcode.pedroPathing.localization.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.DifferentialSwerveDrivetrain;

@Config
@TeleOp(group = "Tests")
public class differential extends LinearOpMode {
    DifferentialSwerveDrivetrain drive = new DifferentialSwerveDrivetrain(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        drive.initialize();

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            drive.singleJoyStickPID(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            telemetry.update();
        }
    }
}