package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
@Config
@TeleOp(name = "Camera Tune",group = "TeleOp")
public class sampleCameraTune extends LinearOpMode {
    OpenCvCamera camera;
    final SampleDetectionPipeline sampleDetectionPipeline = new SampleDetectionPipeline(true);

    public static int x = 0, y = 0;
    public static boolean red, yellow, blue;
    double[] bestInfo;

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.setPipeline(sampleDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {camera.startStreaming(800,448, OpenCvCameraRotation.SIDEWAYS_LEFT);}

            @Override
            public void onError(int errorCode) {}
        });
        telemetry.setMsTransmissionInterval(25);

        FtcDashboard.getInstance().startCameraStream(camera, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        while (!isStarted() && !isStopRequested()) {
            ArrayList<SampleDetectionPipeline.Sample> currentDetections = sampleDetectionPipeline.getDetectedStones();
            if (gamepad1.a) {
                telemetry.addLine("red");
            }
            else if (gamepad1.b) {
                sampleDetectionPipeline.desiredColor = SampleDetectionPipeline.BLUE;
                telemetry.addLine("blue");
            }
            else if (gamepad1.x) {
                sampleDetectionPipeline.desiredColor = SampleDetectionPipeline.YELLOW;
                telemetry.addLine("yellow");
            }
//            if (sampleDetectionPipeline.bestSample != null) {
//                if (sampleDetectionPipeline.bestSample.cameraPosition != null) {
//                    telemetry.addData("angle", sampleDetectionPipeline.bestSample.grabAngle);
//                    telemetry.addData("x", sampleDetectionPipeline.bestSample.cameraPosition.x);
//                    telemetry.addData("y", sampleDetectionPipeline.bestSample.cameraPosition.y);
//                }
//            }
//            bestInfo = sampleDetectionPipeline.getBestSampleInformation(currentDetections, SampleDetectionPipeline.RED);
//                telemetry.addData("x", sample.actualX);
//                telemetry.addData("y", sample.actualY);
//                telemetry.addData("standard y", 28.5 * Math.tan(Math.toRadians(43.13)));
//                telemetry.addData("cos", 28.5 / Math.cos(Math.toRadians(43.13)));
//                telemetry.addData("standard x", (28.5 / Math.cos(Math.toRadians(43.13)))*Math.tan(Math.toRadians(sample.cameraXAngle)));
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetry.update();
        }
    }
}
