package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathBuilder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

/**
 * TODO: documentation
 */
public abstract class rootOpMode extends LinearOpMode {
    protected Follower follower;
    protected Intake intake;
    protected Outtake outtake;
    protected OpenCvCamera camera;
    protected final SampleDetectionPipeline samplePipeline = new SampleDetectionPipeline(true);

    protected double[] bestSampleInformation;

    protected enum intakeStates {IDLE, EXTEND, FIND, ROTATE, DOWN, MOVE, GRAB, CHECK, PRE_DONE, DONE, WRONG}
    protected intakeStates intakeState = intakeStates.IDLE;

    protected enum transferStates{IDLE, UP, WAIT, DOWN, TRANSFER, PRE_DONE, DONE}
    protected transferStates transferState = transferStates.IDLE;

    protected double intakeTimer, transferTimer, currentSlideCM = 0, transferDelay;

    protected boolean TeleOp, changingMode;

    protected Scalar allianceColor = SampleDetectionPipeline.BLUE;

    protected PathChain path1, path2, path3, path4, path5, path6, path7, path8, path9, path10, path11;
    /**
     * TODO: documentation
     * @param TeleOp
     */
    protected void initialize(boolean TeleOp) {
        this.TeleOp = TeleOp;
        follower = new Follower(hardwareMap);
        hardware.reduceHardwareCalls = false;
        intake = new Intake(hardwareMap, TeleOp);
        outtake = new Outtake(hardwareMap, TeleOp);
        hardware.reduceHardwareCalls = true;
        hardware.motors.hook.initMotor(hardwareMap);
        hardware.motors.hook.dcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        /*
            Sets the hubs to using BulkReads. Any read will read all non-I2C sensors from a hub at once.
         */
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /*
            This line makes the telemetry available for FTC Dashboard by ACME Robotics.
         */
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(25);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(640,360, OpenCvCameraRotation.UPSIDE_DOWN);
                camera.setPipeline(samplePipeline);
            }

            @Override
            public void onError(int errorCode) {}
        });
        FtcDashboard.getInstance().startCameraStream(camera, 30);

        if (!TeleOp) {
            specimenPaths();
            return;
        }
        wideCamera();
        follower.startTeleOpDrive();
    }

    protected void specimenPaths() {
        double scoreSpecimenX = 39.701;
        Point specimenPickUp = new Point(8.455, 32.235, Point.CARTESIAN);

        path1 = follower.pathBuilder().addPath(
            // Score first specimen
            new BezierLine(
                new Point(8.191, 56.279, Point.CARTESIAN),
                new Point(scoreSpecimenX, 67.738, Point.CARTESIAN)))
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();
        path2 = follower.pathBuilder()
            .addPath(
                // Get behind first blue sample
                new BezierCurve(
                    new Point(scoreSpecimenX, 67.738, Point.CARTESIAN),
                    new Point(3.963, 45.446, Point.CARTESIAN),
                    new Point(13.211, 26.422, Point.CARTESIAN),
                    new Point(69.226, 43.068, Point.CARTESIAN),
                    new Point(60.242, 28.272, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                    // Score in wing
                    new BezierLine(
                        new Point(60.242, 28.272, Point.CARTESIAN),
                        new Point(17.703, 28.272, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                    // Score second blue sample in wing
                    new BezierCurve(
                        new Point(17.703, 28.272, Point.CARTESIAN),
                        new Point(75.567, 33.292, Point.CARTESIAN),
                        new Point(75.303, 8.191, Point.CARTESIAN),
                        new Point(16.382, 17.439, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                    // Score third blue sample in wing
                    new BezierCurve(
                        new Point(16.382, 17.439, Point.CARTESIAN),
                        new Point(79.266, 24.044, Point.CARTESIAN),
                        new Point(75.567, 1.321, Point.CARTESIAN),
                        new Point(16.117, 10.833, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                    // Get second specimen
                    new BezierCurve(
                        new Point(16.117, 10.833, Point.CARTESIAN),
                        new Point(36.462, 29.593, Point.CARTESIAN),
                        new Point(8.455, 32.235, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path3 = follower.pathBuilder()
                .addPath(
                    // score second specimen
                    new BezierLine(
                        specimenPickUp,
                        new Point(scoreSpecimenX, 69.533, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path4 = follower.pathBuilder()
                .addPath(
                    // Get third specimen
                    new BezierLine(
                        new Point(scoreSpecimenX, 69.533, Point.CARTESIAN),
                        specimenPickUp)
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path5 = follower.pathBuilder()
                .addPath(
                    // Score third specimen
                    new BezierLine(
                        specimenPickUp,
                        new Point(scoreSpecimenX, 71.103, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path6 = follower.pathBuilder()
                .addPath(
                    // Get fourth specimen
                    new BezierLine(
                        new Point(scoreSpecimenX, 71.103, Point.CARTESIAN),
                        specimenPickUp
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path7 = follower.pathBuilder()
                .addPath(
                    // score fourth specimen
                    new BezierLine(
                        specimenPickUp,
                        new Point(scoreSpecimenX, 72.897, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path8 = follower.pathBuilder()
                .addPath(
                    // Park
                    new BezierLine(
                        new Point(scoreSpecimenX, 72.897, Point.CARTESIAN),
                        new Point(6.056, 21.533, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();

        //BasketAuton
        path9 = follower.pathBuilder()
            .addPath(
                // Go to basket
                new BezierLine(
                    new Point(8.854, 80.203, Point.CARTESIAN),
                    new Point(15.624, 128.376, Point.CARTESIAN)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-14))
            .build();
        path10 = follower.pathBuilder()
            .addPath(
                // go to second and third sample
                new BezierLine(
                    new Point(15.624, 128.376, Point.CARTESIAN),
                    new Point(16.145, 135.928, Point.CARTESIAN)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-14), Math.toRadians(0))
            .build();
        path11 = follower.pathBuilder()
            .addPath(
                // Go to observation zone
                new BezierCurve(
                    new Point(16.145, 135.928, Point.CARTESIAN),
                    new Point(62.495, 108.065, Point.CARTESIAN),
                    new Point(60.673, 95.045, Point.CARTESIAN)
                    )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
            .build();
        }

    /**
     * TODO: documentation
     * @param toSpecimenMode
     */
    protected void changeMode(boolean toSpecimenMode) {
        outtake.slidePID(Outtake.slidePositions.CLEARS_ROBOT.getPosition());
        if (!outtake.PIDReady()) return;
        if (toSpecimenMode) hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
        else hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderTransfer);
        changingMode = false;
        //TODO: rumble
    }

    /**
     * TODO: documentation
     */
    protected void transferSample() {
        switch (transferState) {
            case IDLE:
                break;
            case UP:
                outtake.slidePID(Outtake.slidePositions.CLEARS_ROBOT.getPosition());
                intake.slidePID(0);

                if (hardware.servos.wrist.getLastPosition() != hardware.servoPositions.wristTransfer.getPosition()) {
                    transferState = transferStates.WAIT;
                    transferTimer = System.currentTimeMillis();
                    transferDelay = 150;
                    hardware.servos.wrist.setServo(hardware.servoPositions.wristTransfer);
                    break;
                }

                if (!outtake.PIDReady() || !intake.PIDReady()) break;
                transferDelay = 0;

                if (hardware.servos.shoulder.getLastPosition() != hardware.servoPositions.shoulderTransfer.getPosition()) {
                    transferState = transferStates.WAIT;
                    transferTimer = System.currentTimeMillis();
                    transferDelay = 300;
                    hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderTransfer);
                    break;
                }

                if (hardware.servos.outtakeClaw.getLastPosition() != hardware.servoPositions.outtakeRelease.getPosition()) {
                    transferState = transferStates.WAIT;
                    transferTimer = System.currentTimeMillis();
                    transferDelay = 150;
                    hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
                    break;
                }

                transferState = transferStates.DOWN;
                break;
            case WAIT:
                if (transferTimer + transferDelay < System.currentTimeMillis()) transferState = transferStates.DOWN;
                break;
            case DOWN:
                outtake.slidePID(Outtake.slidePositions.TRANSFER.getPosition());
                if (outtake.PIDReady()) transferState = transferStates.TRANSFER;
                break;
            case TRANSFER:
                hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeGrip);
                hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
                transferTimer = System.currentTimeMillis();
                transferState = transferStates.DONE;
                break;
            case PRE_DONE:
                if (transferTimer + 200 < System.currentTimeMillis()) transferState = transferStates.DONE;
                break;
            case DONE:
                transferState = transferStates.IDLE;
                break;
        }
    }

    /**
     * TODO: documentation
     */
    protected void transferSpecimen() {
        transferSample();
    }

    /**
     * TODO: documentation
     * @return if it's picked a good sample or not
     */
    protected boolean chooseSample() {
        telemetry.addData("count", samplePipeline.count);
        if (bestSampleInformation != null) {
            telemetry.addData("x", bestSampleInformation[0]);
            telemetry.addData("y", bestSampleInformation[1]);
            telemetry.addData("angle", bestSampleInformation[2]);
        }
        else telemetry.addLine("No best sample");
        if (samplePipeline.bestSampleInformation == null) return false;
        bestSampleInformation = samplePipeline.bestSampleInformation;

        double total = intake.getSlideLength() + bestSampleInformation[1];

        telemetry.addData("slideLength", intake.getSlideLength());
        telemetry.addData("arm Length", intake.armLength*Math.sin(Math.acos(bestSampleInformation[0]/intake.armLength)));
        telemetry.addData("total", total);
        telemetry.addData("target", total - intake.armLength*Math.sin(Math.acos(bestSampleInformation[0]/intake.armLength)));

        //if it's a good sample
        if (Math.abs(bestSampleInformation[0]) > intake.armLength) return false;
        return !intake.tooLong(intake.predictRobotLength(intake.getSlideLength(), bestSampleInformation[1]));
    }

    /**
     * TODO: documentation
     * @return
     */
    protected boolean checkSample() {
        return true;
    }

    /**
     * TODO: documentation
     * @param reset only for TeleOp
     * @param powerOrPos power only for TeleOp
     */
    protected void intakeSequence(boolean reset, double powerOrPos) {
        switch (intakeState) {
            case IDLE:
                if (intake.PIDReady()) break;
                else intake.slidePID(0);
                break;
            case EXTEND:
                if (TeleOp) {
                    hardware.motors.intake.setPower(powerOrPos);
                }
                else {
                    intake.slidePID(powerOrPos);
                    if (intake.PIDReady()) {
                        intakeState = intakeStates.FIND;
                        sampleCamera();
//                        camera.resumeViewport();
                        hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
                    }
                }
                break;
            case FIND:
                if (chooseSample()) {
                    intakeState = intakeStates.DOWN;
//                    camera.pauseViewport();
                    intake.wristToAngle(Math.toRadians(bestSampleInformation[2]));
                }
                break;
            case DOWN:
                currentSlideCM = hardware.motors.intake.dcMotorEx.getCurrentPosition() / intake.ticksPerCM;
                intake.slideCM(currentSlideCM + bestSampleInformation[0]);
                intake.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
                intakeTimer = System.currentTimeMillis();
                break;
            case MOVE:
                intake.slideCM(currentSlideCM + bestSampleInformation[0]);
                if (intakeTimer + 200 < System.currentTimeMillis()) {
                    intake.elbowYDistance(bestSampleInformation[1]);
                    intake.wristToAngle(bestSampleInformation[2]);
                    intakeTimer = System.currentTimeMillis();
                    intakeState = intakeStates.GRAB;
                }
                break;
            case GRAB:
                intake.slideCM(currentSlideCM + bestSampleInformation[0]);
                if (intakeTimer + 200 < System.currentTimeMillis() && intake.PIDReady()) {
                    hardware.servos.intake.setServo(hardware.servoPositions.intakeGrip);
                    intakeState = intakeStates.CHECK;
                }
                break;
            case CHECK:
                if (checkSample()) {
                    intakeState = intakeStates.DONE;
                    intake.slidePID(0);
                    intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
                } else {
                    intake.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
                    intakeState = intakeStates.WRONG;
                }
                intakeTimer = System.currentTimeMillis();
                break;
            case PRE_DONE:
                intake.slidePID(0);
                if (intakeTimer + 200 < System.currentTimeMillis()) intakeState = intakeStates.DONE;
                break;
            case DONE:
                intakeState = intakeStates.IDLE;
            case WRONG:
                if (intakeTimer + 200 < System.currentTimeMillis()) {
                    hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
                    intakeState = intakeStates.FIND;
                }
                break;
        }
        if (reset) {
            intakeState = intakeStates.IDLE;
            intake.slidePID(0);
            intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
            hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
        }
    }

    protected void chooseAlliance() {
        if (gamepad1.x) {
            allianceColor = SampleDetectionPipeline.BLUE;
            telemetry.addLine("blue");
            telemetry.update();
        }
        if (gamepad1.b) {
            allianceColor = SampleDetectionPipeline.RED;
            telemetry.addLine("red");
            telemetry.update();
        }
    }

    /**
     * TODO: documentation
     */
    protected void sampleCamera() {
        intake.setElbow(hardware.servoPositions.cameraDown.getDifferential());
        hardware.servos.wrist.setServo(hardware.servoPositions.wristSampleCamera);
        samplePipeline.cameraZPos = 25.8;
        samplePipeline.cameraAlpha = 0.0;
        samplePipeline.cameraXPos = 8.3;
        samplePipeline.AREA_LOWER_LIMIT = 9000;
        //TODO: change maximum area
    }

    /**
     * TODO: documentation
     */
    protected void wideCamera() {
        intake.setElbow(hardware.servoPositions.cameraWide.getDifferential());
        hardware.servos.wrist.setServo(hardware.servoPositions.wristSampleCamera);
        //TODO: set to 60 degrees to fit below low chamber.
        samplePipeline.cameraZPos = 28.5;
        samplePipeline.cameraAlpha = 60.0;
        samplePipeline.cameraXPos = -3.0;
        //TODO: change maximum area
    }

    /**
     * TODO: documentation
     */
    protected void specimenCamera() {
        intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
        hardware.servos.wrist.setServo(hardware.servoPositions.wristSpecimenCamera);
        //TODO: change maximum area and other variables
    }
}