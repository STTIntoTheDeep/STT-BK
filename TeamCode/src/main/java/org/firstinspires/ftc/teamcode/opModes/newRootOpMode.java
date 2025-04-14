package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelDeadlineGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.command.utility.conditionals.BlockingConditionalCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.gamepad.Button;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Pose;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.FollowPath;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;

import java.util.List;

/**
 * TODO: documentation of the entire class
 */
public class newRootOpMode extends NextFTCOpMode {
    public newRootOpMode() {
        super(Arm.INSTANCE,
                Camera.INSTANCE,
                Elbow.INSTANCE,
                IntakeClaw.INSTANCE,
                OuttakeClaw.INSTANCE,
                Slides.INSTANCE,
                Wrist.INSTANCE
        );
    }
    protected Command driverControlled;

    protected final Point startPoint = new Point(8.0, 63.5, Point.CARTESIAN);
//    protected final Pose startPose = new Pose(0.0, 0.0, Math.toRadians(0.0)),
//            finishPose = new Pose(20.0, 0.0, Math.toRadians(90.0));

    protected PathChain move, back, path1, path2, path3, path4, path5, path6, path7;

    protected Follower follower;

    protected boolean TeleOp;
    protected static boolean redAlliance = false;

    protected void buildPaths() {
//        move = follower.pathBuilder()
//                .addPath(new BezierLine(new Point(startPose), new Point(finishPose)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), finishPose.getHeading())
//                .build();
//        back = follower.pathBuilder()
//                .addPath(new BezierLine(startPoint, new Point(-32,startPoint.getY())))
//                .setConstantHeadingInterpolation(0)
//                .build();
        path1 = follower.pathBuilder() //start to submersible
                .addPath(new BezierCurve(
                        startPoint,
                        new Point(14.355140186915888,69.08411214953271, Point.CARTESIAN),
                        new Point(42, 68.5, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path2 = follower.pathBuilder() //submersible to back of first sample
                .addPath(new BezierCurve(
                        path1.getPath(0).getLastControlPoint(),
                        new Point(14.843, 63.537, Point.CARTESIAN),
                        new Point(4.687, 13, Point.CARTESIAN),
                        new Point(82, 50, Point.CARTESIAN),
                        new Point(52, 30, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndVelocityConstraint(8)
                .setPathEndTranslationalConstraint(3)
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierLine(
                        new Point(52, 30, Point.CARTESIAN),
                        new Point(20, 30, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndVelocityConstraint(8)
                .setPathEndTranslationalConstraint(3)
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierCurve(
                        new Point(20, 30, Point.CARTESIAN),
                        new Point(68.5, 30, Point.CARTESIAN),
                        new Point(52, 20, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndVelocityConstraint(8)
                .setPathEndTranslationalConstraint(3)
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierLine(
                        new Point(52, 20, Point.CARTESIAN),
                        new Point(20, 20, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndVelocityConstraint(8)
                .setPathEndTranslationalConstraint(3)
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierCurve(
                        new Point(20, 20, Point.CARTESIAN),
                        new Point(68.5, 20, Point.CARTESIAN),
                        new Point(52, 14, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndVelocityConstraint(8)
                .setPathEndTranslationalConstraint(3)
                .setPathEndTimeoutConstraint(0)
                .addPath(new BezierLine(
                        new Point(52, 14, Point.CARTESIAN),
                        new Point(18, 14, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .setPathEndVelocityConstraint(8)
                .setPathEndTranslationalConstraint(3)
                .setPathEndTimeoutConstraint(0)
                .build();
//        path3 = follower.pathBuilder() //first sample to second sample
//                .addPath(new BezierCurve(
//                        path1.getPath(0).getLastControlPoint(),
//                        new Point(23.776, 59.215, Point.CARTESIAN),
//                        new Point(23.551, 39.701,  Point.CARTESIAN)))
//                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-40))
//                .setPathEndTimeoutConstraint(4.5)
//                .build();
//        path4 = follower.pathBuilder() //second sample to third sample
//                .addPath(new BezierLine(
//                        new Point(58.590, 25, Point.CARTESIAN),
//                        new Point(22, 25, Point.CARTESIAN)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .addPath(new BezierCurve(
//                        new Point(22, 25, Point.CARTESIAN),
//                        new Point(74.213, 25, Point.CARTESIAN),
//                        new Point(58.850, 18, Point.CARTESIAN)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
        path5 = follower.pathBuilder() //observation zone to obtaining specimen
                .addPath(new BezierCurve(
                        path2.getPath(5).getLastControlPoint(),
                        new Point(34.633, 31.248, Point.CARTESIAN),
                        new Point(9.374, 51.559, Point.CARTESIAN),
                        new Point(3, 36, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path6 = follower.pathBuilder() //scoring specimen
                .addPath(new BezierCurve(
                        new Point(3, 30, Point.CARTESIAN),
                        new Point(14.061, 65.881, Point.CARTESIAN),
                        new Point(44, 72, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path7 = follower.pathBuilder() //returning to grab specimen from submersible
                .addPath(new BezierCurve(
                        new Point(44, 72, Point.CARTESIAN),
                        new Point(5.989, 60.933, Point.CARTESIAN),
                        new Point(3, 30, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    protected static void alliance(boolean red, boolean blue) {
        if (blue) redAlliance = false;
        else if (red) redAlliance = true;
        if (redAlliance) Camera.INSTANCE.samplePipeline.desiredColor = SampleDetectionPipeline.RED;
        else Camera.INSTANCE.samplePipeline.desiredColor = SampleDetectionPipeline.BLUE;
        OpModeData.telemetry.addData("redAlliance", redAlliance);
        OpModeData.telemetry.update();
    }

    protected static Command grabSpecimen() {
        return new SequentialGroup(
                Arm.INSTANCE.toDown(),
                OuttakeClaw.INSTANCE.closeWhenSample()
        );
    }

    protected static Command scoreSpecimen() {
        return new SequentialGroup(new WaitUntil(hardware::touchingSubmersible),
                Arm.INSTANCE.toHigh()
                ,new ParallelDeadlineGroup(new SequentialGroup(OuttakeClaw.INSTANCE.clear(), new Delay(8.0)), Arm.INSTANCE.holdPosition())
        );
    }

    protected static Command resetArm() {
        return new ParallelGroup(Arm.INSTANCE.toDown(),OuttakeClaw.INSTANCE.open());
    }

    protected static Command fullArm() {return new SequentialGroup(scoreSpecimen(),new Delay(0.5),resetArm());}

    protected Command cameraDown() {
        //TODO: does this work? Or InstantCommand?
        Camera.INSTANCE.cameraDownValues();

        return new ParallelGroup(
                Elbow.INSTANCE.cameraDown(),
                Wrist.INSTANCE.cameraToFront()
        );
    }

    protected Command cameraWide() {
        //TODO: does this work? Or InstantCommand?
        Camera.INSTANCE.cameraWideValues();

        return new ParallelGroup(
                Elbow.INSTANCE.cameraWide(),
                Wrist.INSTANCE.cameraToFront()
        );
    }

    protected Command intakeSimple(double cm){
        return new SequentialGroup(locateSampleSimple(cm), grabSequence());
    }

    protected Command locateSampleSimple() {
        //Move until you've found a good one
        return new SequentialGroup(
                new InstantCommand(() -> Camera.INSTANCE.samplePipeline.externalBestSampleInformation = null),
                new WaitUntil(() -> hardware.getSlideLength() > 20),
                cameraDown(),
                new Delay(0.075),
                Camera.INSTANCE.locateSampleSimple(),
                IntakeClaw.INSTANCE.open()
        );
    }

    protected Command locateSampleSimple(double cm) {
        //Move until you've found a good one
        return new SequentialGroup(
                cameraDown(),
                Slides.INSTANCE.toCM(cm),
                new Delay(0.075),
                Camera.INSTANCE.locateSampleSimple(),
                IntakeClaw.INSTANCE.open()
        );
    }

    protected Command grabSequenceWithCheck() {
        return new BlockingConditionalCommand(
                () -> Camera.INSTANCE.samplePipeline.externalBestSampleInformation != null,
                this::grabSequence,
                NullCommand::new
        );
    }

    protected Command grabSequence() {
        return new SequentialGroup(
                //Move elbow down
                new ParallelGroup(
                        IntakeClaw.INSTANCE.open(),
                        Wrist.INSTANCE.toAngle(Math.toRadians(Camera.INSTANCE.samplePipeline.externalBestSampleInformation[2]) + 0.5*Math.PI - Math.acos(Camera.INSTANCE.samplePipeline.externalBestSampleInformation[0]/hardware.armLength)),
                        Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential()),
                        Slides.INSTANCE.toCM(Camera.INSTANCE.slideTarget)),
                //Move elbow sideways
                Elbow.INSTANCE.yDistance(Camera.INSTANCE.samplePipeline.externalBestSampleInformation[0]).thenWait(0.25).asDeadline(Slides.INSTANCE.toCM(Camera.INSTANCE.slideTarget)),
                new Delay(0.25),
                //Grab sample
                IntakeClaw.INSTANCE.close().asDeadline(Slides.INSTANCE.toCM(Camera.INSTANCE.slideTarget)),
                new Delay(0.25),
                //Go back
                Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential()).and(Wrist.INSTANCE.toTransfer()),
                retractIntake()
        );
    }

    protected Command retractIntake() {
        return new SequentialGroup(
                new BlockingConditionalCommand(
                        () -> hardware.getSlideLength() < 20,
                        () -> Slides.INSTANCE.toCM(20),
                        NullCommand::new
                ),
                //Move elbow up
                new ParallelGroup(
                        Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowTransfer.getDifferential()),
                        Wrist.INSTANCE.toTransfer()
                ),
                new Delay(0.7),
                //Retract slides
                Slides.INSTANCE.toPosition(0)
        );
    }

    protected Command dropSample() {
        return new SequentialGroup(
                Slides.INSTANCE.toCM(20),
                Elbow.INSTANCE.cameraDown(),
                new Delay(1.0),
                Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowDrop.getDifferential()),
                Slides.INSTANCE.toCM(10)
        );
    }

    /**
     * @param followPath
     * @return
     */
    public Command driveToSubmersible(FollowPath followPath) {
        return new ParallelDeadlineGroup(
                new WaitUntil(hardware::touchingSubmersible),
                followPath
        );
    }
    /**
     * @param followPath
     * @return
     */
    public static Command driveToPickup(FollowPath followPath) {
        return new ParallelDeadlineGroup(
                new WaitUntil(hardware::touchingBorder),
                followPath);
    }

    //TODO: see if we can event-base things with this
    public Button touchingSubmersible() {return new Button(hardware::touchingSubmersible);}

    protected void initOpMode(boolean TeleOp) {
        this.TeleOp = TeleOp;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        OpModeData.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OpModeData.telemetry.setMsTransmissionInterval(25);

        FtcDashboard.getInstance().startCameraStream(hardware.camera, 30);

        hardware.initSensors(hardwareMap);

        if (TeleOp) {
            hardware.motors.leftFront.initMotor(hardwareMap);
            hardware.motors.leftBack.initMotor(hardwareMap);
            hardware.motors.rightFront.initMotor(hardwareMap);
            hardware.motors.rightBack.initMotor(hardwareMap);
            MotorEx[] motors = new MotorEx[] {
                    new MotorEx(hardware.motors.leftFront.dcMotorEx),
                    new MotorEx(hardware.motors.rightFront.dcMotorEx),
                    new MotorEx(hardware.motors.leftBack.dcMotorEx),
                    new MotorEx(hardware.motors.rightBack.dcMotorEx)
            };
            driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1());
            return;
        }
        OuttakeClaw.INSTANCE.close().invoke();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startPoint.getX(), startPoint.getY()));
        buildPaths();
    }
}