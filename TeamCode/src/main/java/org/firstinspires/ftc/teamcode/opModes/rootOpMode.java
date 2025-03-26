package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Pose;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;
import org.opencv.core.Scalar;

import java.util.List;

/**
 * TODO: documentation
 */
public abstract class rootOpMode extends LinearOpMode {
    protected Follower follower;
    protected Intake intake;
    protected Outtake outtake;
    protected final SampleDetectionPipeline samplePipeline = new SampleDetectionPipeline(true);

    protected double[] bestSampleInformation;

    protected enum intakeStates {IDLE, FIND, ROTATE, DOWN, MOVE, GRAB, BACK, CHECK, PRE_DONE, DONE, WRONG}
    protected intakeStates intakeState = intakeStates.IDLE;

    protected double continueTime, slideTarget, backPower = -0.25, lastLeftPos, lastRightPos;

    protected boolean TeleOp, intakeOpen = true, outtakeOpen;

    protected Scalar allianceColor = SampleDetectionPipeline.BLUE;

    protected PathChain path1, path2, path3, path4, path5, path6, path7, path8;
    /**
     * TODO: documentation
     * @param TeleOp
     */
    protected void initialize(boolean TeleOp) {
        this.TeleOp = TeleOp;
        follower = new Follower(hardwareMap);
        intake = new Intake(hardwareMap, TeleOp);
        outtake = new Outtake(hardwareMap, TeleOp);
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

        hardware.initCamera(hardwareMap, samplePipeline);
        FtcDashboard.getInstance().startCameraStream(hardware.camera, 30);

        if (!TeleOp) {
            //TODO: do servo position shit maybe? move that from init
            return;
        }
        hardware.reduceHardwareCalls = false;
        wideCamera();
        hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
        hardware.reduceHardwareCalls = true;
        follower.startTeleOpDrive();
    }

    protected void specimenPaths() {
        double scoreSpecimenX = 39;
        Point specimenPickUp = new Point(16 , 32.235, Point.CARTESIAN);
        follower.setMaxPower(0.6);

        follower.setStartingPose(new Pose(8.191, 56.279, 0));
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
                    new Point(65.527, 36.198, Point.CARTESIAN),
                    new Point(60.242, 27, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                    // Score in wing
                    new BezierLine(
                        new Point(60.242, 27, Point.CARTESIAN),
                        new Point(21, 27, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                    // Score second blue sample in wing
                    new BezierCurve(
                        new Point(21, 27, Point.CARTESIAN),
                        new Point(75.567, 33.292, Point.CARTESIAN),
                        new Point(75.303, 8.191, Point.CARTESIAN),
                        new Point(21, 18, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                    // Score third blue sample in wing
                    new BezierCurve(
                        new Point(21, 18, Point.CARTESIAN),
                        new Point(79.266, 28, Point.CARTESIAN),
                        new Point(75.567, 5, Point.CARTESIAN),
                        new Point(21, 10, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(
                    // Get second specimen
                    new BezierCurve(
                        new Point(21, 10, Point.CARTESIAN),
                        new Point(36.462, 29.593, Point.CARTESIAN),
                        specimenPickUp
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path3 = follower.pathBuilder()
            .addPath(
                // score second specimen
                new BezierLine(
                    specimenPickUp,
                    new Point(scoreSpecimenX, 70, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();
            follower.setMaxPower(1.0);
        path4 = follower.pathBuilder()
            .addPath(
                // Get third specimen
                new BezierLine(
                    new Point(scoreSpecimenX, 70, Point.CARTESIAN),
                    specimenPickUp)
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();
            follower.setMaxPower(1.0);
        path5 = follower.pathBuilder()
            .addPath(
                // Score third specimen
                new BezierLine(
                    specimenPickUp,
                    new Point(scoreSpecimenX, 74, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();
            follower.setMaxPower(1.0);
        path6 = follower.pathBuilder()
            .addPath(
                // Get fourth specimen
                new BezierLine(
                    new Point(scoreSpecimenX, 74, Point.CARTESIAN),
                    specimenPickUp
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();
            follower.setMaxPower(1.0);
        path7 = follower.pathBuilder()
            .addPath(
                // score fourth specimen
                new BezierLine(
                    specimenPickUp,
                    new Point(scoreSpecimenX, 76, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .build();
            follower.setMaxPower(1.0);
        path8 = follower.pathBuilder()
                .addPath(
                    // Park
                    new BezierLine(
                        new Point(scoreSpecimenX, 76, Point.CARTESIAN),
                        new Point(6.056, 21.533, Point.CARTESIAN)
                    )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0)).build();
            follower.setMaxPower(1.0);
        }
    protected void samplePaths() {
        follower.setStartingPose(new Pose(8, 102, 0));
        path1 = follower.pathBuilder()
            .addPath(
                // Go to basket
                new BezierLine(
                    new Point(8, 102, Point.CARTESIAN),
                    new Point(8, 125, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(0))
            .addPath(new BezierLine(
                new Point(8, 125, Point.CARTESIAN),
                new Point(8, 125, Point.CARTESIAN)
                )
            )
            .setConstantHeadingInterpolation(-20)
            .build();
        path2 = follower.pathBuilder()
            .addPath(
                // go to second and third sample
                new BezierLine(
                    new Point(8, 125, Point.CARTESIAN),
                    new Point(10, 130, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(-15), Math.toRadians(0))
            .build();
        path3 = follower.pathBuilder()
            .addPath(
                // Go to observation zone
                new BezierCurve(
                    new Point(15.061, 134.752, Point.CARTESIAN),
                    new Point(64, 112, Point.CARTESIAN),
                    new Point(60.673, 95.045, Point.CARTESIAN)
                )
            )
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
            .build();
    }

    /**
     * Controls the outtake
     * Not allowed to move the outtake while transferring or changing mode
     * TODO: documentation
     */
    protected void TeleOpOuttake(double power, boolean toggle) {
        // Normally you'd want a rising edge detector here but the hardware wrapper already covers that.
        // This code automatically moves the shoulder to the right position
        lastLeftPos = outtake.leftPos;
        lastRightPos = outtake.rightPos;
        outtake.slidesWithinLimits(power, 1400);
        // Outtake claw toggle
        if (toggle) {
            hardware.servos.outtakeClaw.setServo(
                    (hardware.servos.outtakeClaw.getLastPosition() == hardware.servoPositions.outtakeGrip.getPosition())
                            ? hardware.servoPositions.outtakeRelease : hardware.servoPositions.outtakeGrip);
        }
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

    public void manualIntakeSequence(boolean next, double power) {
        if (next) {
            if (intakeState == intakeStates.IDLE) {
                samplePipeline.desiredColor = SampleDetectionPipeline.YELLOW;
                intakeState = intakeStates.FIND;
                intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential()[0], 0.34);
            } else if (intakeState == intakeStates.FIND) {
                intakeState = intakeStates.DOWN;
                intake.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
            } else if (intakeState == intakeStates.DOWN) {
                hardware.servos.intake.setServo(hardware.servoPositions.intakeGrip);
                continueTime = System.currentTimeMillis() + 300;
                intakeState = intakeStates.GRAB;
            } else if (intakeState == intakeStates.DONE) {
                intakeState = intakeStates.IDLE;
                hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
            }
        }

        switch (intakeState) {
            case IDLE:
                if (!intake.PIDReady()) intake.slidePID(0);
                break;
            case FIND:
            case DOWN:
            case DONE:
                intake.slideWithinLimits(Math.max(power, -0.5));
                break;
            case GRAB:
                intake.slideWithinLimits(Math.max(power, -0.5));
                if (continueTime < System.currentTimeMillis()) {
                    intakeState = intakeStates.BACK;
                    continueTime = System.currentTimeMillis() + intake.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
                    hardware.servos.wrist.setServo(hardware.servoPositions.wristTransfer);
                }
                break;
            case BACK:
                if (continueTime < System.currentTimeMillis()) {
                    intakeState = intakeStates.PRE_DONE;
                    continueTime = System.currentTimeMillis() + intake.setElbow(hardware.servoPositions.cameraDown.getDifferential());
                }
                break;
            case PRE_DONE:
                intake.slidePID(0);
                if (continueTime < System.currentTimeMillis() && hardware.motors.intake.dcMotorEx.getCurrentPosition() < 40) {
                    intakeState = intakeStates.DONE;
                }
                break;
        }
    }

    /**
     * TODO: documentation
     * @param next
     * @param reset
     */
    public void simpleIntakeSequence(boolean next, boolean reset, double powerOrPos) {
        if (next) {
            if (intakeState == intakeStates.IDLE) {
                samplePipeline.desiredColor = SampleDetectionPipeline.YELLOW;
                intakeState = intakeStates.FIND;
                samplePipeline.saveRAM = false;
                sampleCamera();
            } else if (intakeState == intakeStates.FIND && bestSampleInformation != null) {
                samplePipeline.saveRAM = true;
                hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
                continueTime = 500 + System.currentTimeMillis() + intake.wristToAngle(Math.toRadians(bestSampleInformation[2]) + 0.5*Math.PI - Math.acos(bestSampleInformation[0]/intake.armLength));
                intake.setElbow(new double[] {hardware.servoPositions.elbowCentered.getDifferential()[0], 0.0});
                slideTarget = intake.getSlideLength() + bestSampleInformation[1] - intake.armLength*Math.sin(Math.acos(-bestSampleInformation[0]/intake.armLength));
                intakeState = intakeStates.DOWN;
            } else if (intakeState == intakeStates.DONE) intakeState = intakeStates.IDLE;
        }

        if (reset) {}

        switch (intakeState) {
            case IDLE:
                if (!intake.PIDReady()) intake.slidePID(0);
                break;
            case FIND:
                if (TeleOp) {
                    intake.slideWithinLimits(Math.max(powerOrPos, -0.5));
                } else intake.slidePID(powerOrPos);
                if (chooseSample()) {
                    if (TeleOp) {
                        //TODO: rumble
                    } else intakeState = intakeStates.DOWN;
                }
                break;
            case DOWN:
                intake.slideCM(slideTarget);
                if (continueTime < System.currentTimeMillis()) {
                    continueTime = System.currentTimeMillis() + intake.elbowYDistance(bestSampleInformation[0]) + 250;
                    intakeState = intakeStates.MOVE;
                }
                break;
            case MOVE:
                intake.slideCM(slideTarget);
                if (intake.PIDReady() && continueTime < System.currentTimeMillis()) {
                    hardware.servos.intake.setServo(hardware.servoPositions.intakeGrip);
                    continueTime = System.currentTimeMillis() + 300;
                    intakeState = intakeStates.GRAB;
                }
                break;
            case GRAB:
                if (continueTime < System.currentTimeMillis()) {
                    intakeState = intakeStates.BACK;
                    continueTime = System.currentTimeMillis() + intake.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
                    hardware.servos.wrist.setServo(hardware.servoPositions.wristTransfer);
                }
                break;
            case BACK:
                if (continueTime < System.currentTimeMillis()) {
                    intakeState = intakeStates.PRE_DONE;
                    continueTime = System.currentTimeMillis() + intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
                }
                break;
            case PRE_DONE:
                intake.slidePID(0);
                if (continueTime < System.currentTimeMillis() && hardware.motors.intake.dcMotorEx.getCurrentPosition() < 40) {
                    intakeState = intakeStates.DONE;
                }
                break;
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
        hardware.cameraZPos = 28.9;
        hardware.cameraAlpha = 0.0;
        hardware.cameraXPos = 6.8;
        hardware.cameraYPos = -0.6;
        samplePipeline.AREA_LOWER_LIMIT = 30000;
        //TODO: change maximum area
    }

    /**
     * TODO: documentation
     */
    protected void wideCamera() {
        intake.setElbow(hardware.servoPositions.cameraWide.getDifferential());
        hardware.servos.wrist.setServo(hardware.servoPositions.wristSampleCamera);
        //TODO: set to 60 degrees to fit below low chamber.
        hardware.cameraZPos = 28.5;
        hardware.cameraAlpha = 60.0;
        hardware.cameraXPos = -3.0;
        //TODO: change minimum and maximum area
    }

    /**
     * TODO: documentation
     */
    protected void specimenCamera() {
        intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
        hardware.servos.wrist.setServo(hardware.servoPositions.wristSpecimenCamera);
        //TODO: change maximum area and other variables
    }

    //TODO: save all servo positions and location to an XML file (see Roadrunner documentation)
}