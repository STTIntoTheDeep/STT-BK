package org.firstinspires.ftc.teamcode.opModes;

import android.graphics.Path;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelDeadlineGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.command.utility.PerpetualCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.gamepad.Button;

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
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;

import java.util.List;

public class newRootOpMode extends NextFTCOpMode {
    public newRootOpMode() {
        super(Arm.INSTANCE,
                Elbow.INSTANCE,
                IntakeClaw.INSTANCE,
                OuttakeClaw.INSTANCE,
                Slides.INSTANCE,
                Wrist.INSTANCE
        );
    }
    protected Command driverControlled;

    protected final Point startPoint = new Point(8.593, 55.725, Point.CARTESIAN);
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
//                .addPath(new BezierLine(new Point(finishPose), new Point(startPose)))
//                .setLinearHeadingInterpolation(finishPose.getHeading(), startPose.getHeading())
//                .build();
        path1 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        startPoint,
                        new Point(24.738, 67.703, Point.CARTESIAN),
                        new Point(39.580, 67.443, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path2 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(39.580, 67.443, Point.CARTESIAN),
                        new Point(14.843, 63.537, Point.CARTESIAN),
                        new Point(4.687, 18.228, Point.CARTESIAN),
                        new Point(91.660, 42.184, Point.CARTESIAN),
                        new Point(58.590, 26.561, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(58.590, 26.561, Point.CARTESIAN),
                        new Point(15.103, 26.561, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(15.103, 26.561, Point.CARTESIAN),
                        new Point(74.474, 26.821, Point.CARTESIAN),
                        new Point(58.590, 16.145, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path4 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(58.590, 16.145, Point.CARTESIAN),
                        new Point(15.103, 16.145, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(15.103, 16.145, Point.CARTESIAN),
                        new Point(74.213, 16.665, Point.CARTESIAN),
                        new Point(58.850, 8.333, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path5 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Point(58.850, 8.333, Point.CARTESIAN),
                        new Point(15.103, 7.812, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addPath(new BezierCurve(
                        new Point(15.103, 7.812, Point.CARTESIAN),
                        new Point(34.633, 31.248, Point.CARTESIAN),
                        new Point(9.374, 51.559, Point.CARTESIAN),
                        new Point(8.593, 25.259, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path6 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(8.593, 25.259, Point.CARTESIAN),
                        new Point(14.061, 65.881, Point.CARTESIAN),
                        new Point(39.320, 69.266, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
        path7 = follower.pathBuilder()
                .addPath(new BezierCurve(
                        new Point(39.320, 69.266, Point.CARTESIAN),
                        new Point(5.989, 60.933, Point.CARTESIAN),
                        new Point(8.854, 23.696, Point.CARTESIAN)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    protected static Command alliance(boolean red, boolean blue) {
        if (blue) redAlliance = false;
        else if (red) redAlliance = true;
        return new InstantCommand(() -> {
            OpModeData.telemetry.addData("redAlliance", redAlliance);
            OpModeData.telemetry.update();
        });
    }

    protected static Command grabSpecimen() {
        return new SequentialGroup(
                Arm.INSTANCE.toDown(),
                OuttakeClaw.INSTANCE.close()
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

    protected static Command cameraDown() {
        hardware.cameraZPos = 28.9;
        hardware.cameraAlpha = 0.0;
        hardware.cameraXPos = 6.8;
        hardware.cameraYPos = -0.6;
        Camera.samplePipeline.AREA_LOWER_LIMIT = 30000;
        return new ParallelGroup(
                Elbow.INSTANCE.cameraDown(),
                Wrist.INSTANCE.cameraToFront()
        );
    }

    public Command intakeSequence() {
        final double[] slideTarget = {0};
        return new SequentialGroup(
                //Move until you've found a good one
                new ParallelDeadlineGroup(
                        new LambdaCommand()
                                .setStart(() -> Camera.samplePipeline.saveRAM = false)
                                .setUpdate(Camera::chooseSample),
                        cameraDown(),
                        (TeleOp) ? Slides.INSTANCE.setPowerWithinLimits(gamepad1.left_trigger - gamepad1.right_trigger) : Slides.INSTANCE.toPosition(500)
                ),
                //Move elbow down
                new ParallelGroup(
                        IntakeClaw.INSTANCE.open(),
                        new InstantCommand(() -> {
                            Camera.samplePipeline.saveRAM = true;
                            slideTarget[0] = hardware.getSlideLength() + Camera.bestSampleInformation[1] - hardware.armLength*Math.sin(Math.acos(-Camera.bestSampleInformation[0]/hardware.armLength));
                        }),
                        Wrist.INSTANCE.toAngle(Math.toRadians(Camera.bestSampleInformation[2]) + 0.5*Math.PI - Math.acos(Camera.bestSampleInformation[0]/hardware.armLength)),
                        Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential()),
                        Slides.INSTANCE.toCM(slideTarget[0])
                ),
                //Move elbow sideways
                Elbow.INSTANCE.yDistance(Camera.bestSampleInformation[0]).thenWait(0.25).asDeadline(Slides.INSTANCE.toCM(slideTarget[0])),
                //Grab sample
                IntakeClaw.INSTANCE.close().asDeadline(Slides.INSTANCE.toCM(slideTarget[0])),
                //Go back
                Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential()).and(Wrist.INSTANCE.toTransfer()),
                //Move elbow up
                Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowTransfer.getDifferential()),
                //Retract slides
                Slides.INSTANCE.toPosition(0)
        );
    }

    public Button touchingSubmersible() {return new Button(hardware::touchingSubmersible);}

    protected void initOpMode(boolean TeleOp) {
        this.TeleOp = TeleOp;

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(25);

        FtcDashboard.getInstance().startCameraStream(hardware.camera, 30);

        hardware.initSensors(hardwareMap);

        if (TeleOp) {
            return;
        }
//        cameraDown().invoke();
        follower = new Follower(hardwareMap);
        follower.setStartingPose(new Pose(startPoint.getX(), startPoint.getY()));
        buildPaths();
    }
}