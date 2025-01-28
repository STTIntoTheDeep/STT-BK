package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.List;

/**
 * TODO: documentation
 */
public abstract class rootOpMode extends LinearOpMode {
    Follower follower;
    Intake intake;
    Outtake outtake;
    OpenCvCamera camera;
    final SampleDetectionPipeline sampleDetectionPipeline = new SampleDetectionPipeline(false);

    ArrayList<SampleDetectionPipeline.Sample> currentDetections;
    double[] bestSampleInformation;

    protected enum intakeStates {IDLE, RELEASE, EXTEND, FIND, DOWN, MOVE, GRAB, CHECK, PRE_DONE, DONE, WRONG}
    protected intakeStates intakeState = intakeStates.IDLE;

    protected enum transferStates{IDLE, UP, WAIT, DOWN, TRANSFER, PRE_DONE, DONE}
    protected transferStates transferState = transferStates.IDLE;

    double intakeTimer, transferTimer, currentSlideCM = 0, transferDelay;

    boolean TeleOp, changingMode;

    Scalar desiredColor = SampleDetectionPipeline.YELLOW;
    Scalar allianceColor = SampleDetectionPipeline.BLUE;

    /**
     * TODO: documentation
     * @param TeleOp
     */
    protected void initialize(boolean TeleOp) {
        this.TeleOp = TeleOp;
        //TODO: follower
        follower = new Follower(hardwareMap);
        intake = new Intake(hardwareMap, TeleOp);
        outtake = new Outtake(hardwareMap, TeleOp);

        /*
            Sets the hubs to using BulkReads. Any read will read all non-I2C sensors from a hub at once.
         */
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

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

        /*
            This line makes the telemetry available for FTC Dashboard by ACME Robotics.
         */
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        if (!TeleOp) return;
        follower.startTeleopDrive();
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
        bestSampleInformation = null;
        currentDetections = sampleDetectionPipeline.getDetectedStones();
        bestSampleInformation = sampleDetectionPipeline.getBestSampleInformation(currentDetections);
        if (bestSampleInformation == null) return false;

        //if it's a good sample
        return bestSampleInformation[0] < 300;
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
                if (intake.PIDReady()) hardware.servos.keepSlide.setServo(hardware.servoPositions.keepSlides);
                else intake.slidePID(0);
                break;
            case RELEASE:
                intakeTimer = System.currentTimeMillis();
                hardware.servos.keepSlide.setServo(hardware.servoPositions.releaseSlides);
                break;
            case EXTEND:
                if (intakeTimer + 200 < System.currentTimeMillis()) {
                    if (TeleOp) {
                        hardware.motors.intake.setPower(powerOrPos);
                    }
                    else {
                        intake.slidePID(powerOrPos);
                        if (intake.PIDReady()) intakeState = intakeStates.FIND;
                    }
                }
                break;
            case FIND:
                if (chooseSample()) intakeState = intakeStates.GRAB;
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
                    intake.elbowToAngle(bestSampleInformation[1]);
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
}
