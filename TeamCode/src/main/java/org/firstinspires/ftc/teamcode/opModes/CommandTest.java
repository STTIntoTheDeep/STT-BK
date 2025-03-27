package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelDeadlineGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.LambdaCommand;
import com.rowanmcalpin.nextftc.core.command.utility.PerpetualCommand;
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

    public static Command a() {
        return                 //Move until you've found a good one
                new ParallelDeadlineGroup(
                        new LambdaCommand()
                                .setStart(() -> Camera.samplePipeline.saveRAM = false)
                                .setUpdate(Camera::chooseSample),
                        cameraDown(),
                        Slides.INSTANCE.setPowerWithinLimits(OpModeData.INSTANCE.getGamepad1().left_trigger - OpModeData.INSTANCE.getGamepad1().right_trigger)
                );
    }

    public static Command b(){
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

    public static Command c(){//Move elbow sideways
        return Elbow.INSTANCE.yDistance(Camera.bestSampleInformation[0]).thenWait(0.25).asDeadline(Slides.INSTANCE.toCM(slideTarget[0]));
    }
    public static Command d(){
        //Grab sample
        return IntakeClaw.INSTANCE.close().asDeadline(Slides.INSTANCE.toCM(slideTarget[0]));
    }
    public static Command e(){
        //Go back
        return Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential()).and(Wrist.INSTANCE.toTransfer());
    }
    public static Command f(){
        //Move elbow up
        return Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
    }
    public static Command g(){
        //Retract slides
        return Slides.INSTANCE.toPosition(0);
    }

    @Override
    public void onInit() {
        initOpMode(false);
        alliance(gamepad1.a, gamepad1.b).invoke();
    }

    @Override
    public void onStartButtonPressed() {
//        new FollowPath(move, follower).invoke();
        gamepadManager.getGamepad1().getDpadUp().setPressedCommand(CommandTest::a);
        gamepadManager.getGamepad1().getDpadLeft().setPressedCommand(CommandTest::b);
        gamepadManager.getGamepad1().getDpadDown().setPressedCommand(CommandTest::c);
        gamepadManager.getGamepad1().getDpadRight().setPressedCommand(CommandTest::d);
        gamepadManager.getGamepad1().getX().setPressedCommand(CommandTest::e);
        gamepadManager.getGamepad1().getA().setPressedCommand(CommandTest::f);
        gamepadManager.getGamepad1().getB().setPressedCommand(CommandTest::g);
    }
}