package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelDeadlineGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.gamepad.Button;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Follower;
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
        super(
                Arm.INSTANCE,
                Elbow.INSTANCE,
                IntakeClaw.INSTANCE,
                OuttakeClaw.INSTANCE,
                Slides.INSTANCE,
                Wrist.INSTANCE
        );
    }
    protected Command driverControlled;

    protected final Pose startPose = new Pose(0.0, 0.0, Math.toRadians(0.0)),
            finishPose = new Pose(20.0, 0.0, Math.toRadians(90.0));

    protected PathChain move, back;

    protected Follower follower;
    protected SampleDetectionPipeline samplePipeline;

    protected boolean TeleOp;

    protected void buildPaths() {
        move = follower.pathBuilder()
                .addPath(new BezierLine(new Point(startPose), new Point(finishPose)))
                .setLinearHeadingInterpolation(startPose.getHeading(), finishPose.getHeading())
                .build();
        back = follower.pathBuilder()
                .addPath(new BezierLine(new Point(finishPose), new Point(startPose)))
                .setLinearHeadingInterpolation(finishPose.getHeading(), startPose.getHeading())
                .build();
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
//                ,OuttakeClaw.INSTANCE.open().asDeadline(Arm.INSTANCE.holdPosition())
//                ,Arm.INSTANCE.toPosition(600).and(Arm.INSTANCE.holdPosition())
//                Arm.INSTANCE.holdPosition().and(
//                        new WaitUntil(() -> !hardware.touchingSubmersible())
//                                .then(new Delay(1.0))),
//                Arm.INSTANCE.toDown()
        );
    }

    protected Command cameraDown() {
        hardware.cameraZPos = 28.9;
        hardware.cameraAlpha = 0.0;
        hardware.cameraXPos = 6.8;
        hardware.cameraYPos = -0.6;
        samplePipeline.AREA_LOWER_LIMIT = 30000;
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
                                .setStart(() -> samplePipeline.saveRAM = false)
                                .setUpdate(Camera::chooseSample),
                        cameraDown(),
                        (TeleOp) ? Slides.INSTANCE.setPowerWithinLimits(gamepad1.left_trigger - gamepad1.right_trigger) : Slides.INSTANCE.toPosition(500)
                ),
                //Move elbow down
                new ParallelGroup(
                        IntakeClaw.INSTANCE.open(),
                        new InstantCommand(() -> {
                            samplePipeline.saveRAM = true;
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
        follower.setStartingPose(startPose);
        buildPaths();
    }
}