package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.command.utility.conditionals.BlockingConditionalCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@TeleOp(name = "Command Test")
public class CommandTest extends newRootOpMode {

    public static int position = 200;

//    private Command routine() {
//        return new PerpetualCommand(new SequentialGroup(
//                Arm.INSTANCE.toPosition(position)
////                new InstantCommand(() -> OpModeData.telemetry.addData("pos", hardware.motors.outtake.dcMotorEx.getCurrentPosition()))
//        ));
//    }

//    private Command routine() {
//        return new SequentialGroup(
//                OuttakeClaw.INSTANCE.open(),
//                new Delay(1.0),
//                new ParallelGroup(
//                        OuttakeClaw.INSTANCE.close(),
//                        IntakeClaw.INSTANCE.close()
//                )
//        );
//    }

    /*
    private Command b(){
        //Move elbow down
        return new ParallelGroup(
                IntakeClaw.INSTANCE.open(),
                Wrist.INSTANCE.toAngle(Math.toRadians(Camera.bestSampleInformation[2]) + 0.5*Math.PI - Math.acos(Camera.bestSampleInformation[0]/hardware.armLength)),
                Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential()),
                Slides.INSTANCE.toCM(Camera.INSTANCE.slideTarget)
        );
    }

    private Command c(){//Move elbow sideways
        return Elbow.INSTANCE.yDistance(Camera.bestSampleInformation[0]).thenWait(0.25).asDeadline(Slides.INSTANCE.toCM(Camera.INSTANCE.slideTarget));
    }
    private Command d(){
        //Grab sample
        return IntakeClaw.INSTANCE.close().asDeadline(Slides.INSTANCE.toCM(Camera.INSTANCE.slideTarget));
    }
    private Command e(){
        //Go back
        return Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential()).and(Wrist.INSTANCE.toTransfer());
    }
    private Command f(){
        //Move elbow up
        return new ParallelGroup(
                Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowTransfer.getDifferential()),
                Wrist.INSTANCE.toTransfer()
        );
    }
    private Command g(){
        //Retract slides
        return Slides.INSTANCE.toPosition(0);
    }
    public Command i(){return new InstantCommand(() -> {OpModeData.telemetry.addLine("doing something");Camera.samplePipeline.saveRAM = false;});}
    public Command j() {
        return new InstantCommand(() -> Camera.INSTANCE.samplePipeline.saveRAM = false);
    }
    public Command k() {
        return new InstantCommand(() -> Camera.INSTANCE.samplePipeline.saveRAM = true);
    }
     */

    protected Command seekSample() {
        return new SequentialGroup(
                Slides.INSTANCE.toCM(0),
                cameraWide(),
                Camera.INSTANCE.locateSampleSimple(),
                Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowTransfer.getDifferential()),
                Slides.INSTANCE.toCM(Camera.INSTANCE.slideTarget),
                cameraDown(),
                new Delay(0.075),
                Camera.INSTANCE.locateSampleSimple(),
                IntakeClaw.INSTANCE.open(),
                grabSequence()
        );
    }

    @Override
    public void onInit() {
        initOpMode(false);
        alliance(gamepad1.a, gamepad1.b);
        Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowTransfer.getDifferential()).invoke();
        Camera.INSTANCE.samplePipeline.saveRAM = false;
//        Elbow.INSTANCE.cameraDown().invoke();
    }

    @Override
    public void onStartButtonPressed() {
//        new FollowPath(move, follower).invoke();
        gamepadManager.getGamepad1().getDpadUp().setPressedCommand(this::locateSampleSimple);
//        gamepadManager.getGamepad1().getDpadLeft().setPressedCommand(this::b);
//        gamepadManager.getGamepad1().getDpadDown().setPressedCommand(this::c);
//        gamepadManager.getGamepad1().getDpadRight().setPressedCommand(this::d);
//        gamepadManager.getGamepad1().getX().setPressedCommand(this::e);
//        gamepadManager.getGamepad1().getA().setPressedCommand(this::f);
//        gamepadManager.getGamepad1().getB().setPressedCommand(this::g);
        gamepadManager.getGamepad1().getY().setPressedCommand(this::cameraDown);
        gamepadManager.getGamepad1().getX().setPressedCommand(this::cameraWide);
//        gamepadManager.getGamepad1().getY().setPressedCommand(this::j);
//        gamepadManager.getGamepad1().getB().setPressedCommand(this::k);
        gamepadManager.getGamepad1().getDpadDown().setPressedCommand(this::grabSequence);
        gamepadManager.getGamepad1().getDpadLeft().setPressedCommand(this::dropSample);
        gamepadManager.getGamepad1().getDpadRight().setPressedCommand(this::retractIntake);
    }
}