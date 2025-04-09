package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "OpenCV Testing")
@Config
public class specimenCameraTune extends LinearOpMode {
    SpecimenDetectionPipeline pipeline = new SpecimenDetectionPipeline(true);

    @Override
    public void runOpMode() {
        hardware.initCamera(hardwareMap, pipeline);
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        FtcDashboard.getInstance().startCameraStream(hardware.camera, 30);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Coordinate", "(" + (int) pipeline.cX + ", " + (int) pipeline.cY + ")");
            telemetry.addData("Distance in Inch", (pipeline.getDistance()));
            telemetry.update();

            // The OpenCV pipeline automatically processes frames and handles detection
        }

        // Release resources
        hardware.camera.stopStreaming();
    }
}