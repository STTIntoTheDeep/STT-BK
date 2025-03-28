package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelDeadlineGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.command.utility.PerpetualCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Config
@TeleOp(name = "Command Test")
public class CommandTest extends newRootOpMode {

    static final double[] slideTarget = {0};

    public static int position = 200;

//    public Command routine() {
//        return new PerpetualCommand(new SequentialGroup(
//                Arm.INSTANCE.toPosition(position)
////                new InstantCommand(() -> OpModeData.telemetry.addData("pos", hardware.motors.outtake.dcMotorEx.getCurrentPosition()))
//        ));
//    }

//    public Command routine() {
//        return new SequentialGroup(
//                OuttakeClaw.INSTANCE.open(),
//                new Delay(1.0),
//                new ParallelGroup(
//                        OuttakeClaw.INSTANCE.close(),
//                        IntakeClaw.INSTANCE.close()
//                )
//        );
//    }

    public Command a() {
        //Move until you've found a good one
        return new ParallelDeadlineGroup(
                new InstantCommand(() -> Camera.samplePipeline.saveRAM = false).then(new WaitUntil(Camera.INSTANCE::chooseSample)),
                cameraDown(),
                Slides.INSTANCE.toCM(15)
        );
    }

    public Command b(){
        //Move elbow down
        return new ParallelGroup(
                IntakeClaw.INSTANCE.open(),
                new InstantCommand(() -> {
                    Camera.samplePipeline.saveRAM = true;
                    slideTarget[0] = hardware.getSlideLength() + Camera.bestSampleInformation[1] - hardware.armLength*Math.sin(Math.acos(-Camera.bestSampleInformation[0]/hardware.armLength));
                }),
                Wrist.INSTANCE.toAngle(Math.toRadians(Camera.bestSampleInformation[2]) + 0.5*Math.PI - Math.acos(Camera.bestSampleInformation[0]/hardware.armLength)),
                Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential()),
                Slides.INSTANCE.toCM(slideTarget[0])
        );
    }

    public Command c(){//Move elbow sideways
        return Elbow.INSTANCE.yDistance(Camera.bestSampleInformation[0]).thenWait(0.25).asDeadline(Slides.INSTANCE.toCM(slideTarget[0]));
    }
    public Command d(){
        //Grab sample
        return IntakeClaw.INSTANCE.close().asDeadline(Slides.INSTANCE.toCM(slideTarget[0]));
    }
    public Command e(){
        //Go back
        return Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential()).and(Wrist.INSTANCE.toTransfer());
    }
    public Command f(){
        //Move elbow up
        return Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
    }
    public Command g(){
        //Retract slides
        return Slides.INSTANCE.toPosition(0);
    }

    @Override
    public void onInit() {
        initOpMode(false);
        OpModeData.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        alliance(gamepad1.a, gamepad1.b).invoke();
    }

    @Override
    public void onStartButtonPressed() {
//        new FollowPath(move, follower).invoke();
        gamepadManager.getGamepad1().getDpadUp().setPressedCommand(this::a);
        gamepadManager.getGamepad1().getDpadLeft().setPressedCommand(this::b);
        gamepadManager.getGamepad1().getDpadDown().setPressedCommand(this::c);
        gamepadManager.getGamepad1().getDpadRight().setPressedCommand(this::d);
        gamepadManager.getGamepad1().getX().setPressedCommand(this::e);
        gamepadManager.getGamepad1().getA().setPressedCommand(this::f);
        gamepadManager.getGamepad1().getB().setPressedCommand(this::g);
    }
}