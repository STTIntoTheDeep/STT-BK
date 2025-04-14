package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Camera;
import org.firstinspires.ftc.teamcode.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
@Disabled
@Config
@TeleOp(name = "Command Drive")
public class CommandDrive extends newRootOpMode {

    public static int position = 200;

    @Override
    public void onInit() {
        initOpMode(true);
        alliance(gamepad1.a, gamepad1.b);
        Camera.INSTANCE.samplePipeline.saveRAM = false;
        OuttakeClaw.INSTANCE.open().invoke();
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled.invoke();

        gamepadManager.getGamepad1().getDpadLeft().setPressedCommand(OuttakeClaw.INSTANCE::outtakeToggle);
        gamepadManager.getGamepad1().getDpadLeft().setHeldCommand(() -> Arm.INSTANCE.setPower(gamepad1.right_trigger - gamepad1.left_trigger));
        gamepadManager.getGamepad1().getDpadUp().setPressedCommand(newRootOpMode::scoreSpecimen);
        gamepadManager.getGamepad1().getDpadUp().setReleasedCommand(newRootOpMode::resetArm);
        gamepadManager.getGamepad1().getLeftBumper().setPressedCommand(newRootOpMode::grabSpecimen);
        gamepadManager.getGamepad1().getLeftBumper().setReleasedCommand(newRootOpMode::fullArm);

        gamepadManager.getGamepad1().getA().setPressedCommand(this::dropSample);
        gamepadManager.getGamepad1().getB().setPressedCommand(this::retractIntake);
        gamepadManager.getGamepad1().getX().setPressedCommand(IntakeClaw.INSTANCE::toggle);
        gamepadManager.getGamepad1().getY().setPressedCommand(Elbow.INSTANCE::toggle);
        gamepadManager.getGamepad1().getRightStick().getButton().setHeldCommand(() -> Slides.INSTANCE.setPowerWithinLimits(-gamepad1.right_stick_y));
        gamepadManager.getGamepad1().getRightBumper().setPressedCommand(this::locateSampleSimple);
        gamepadManager.getGamepad1().getRightBumper().setHeldCommand(() -> Slides.INSTANCE.setPowerWithinLimits(-gamepad1.right_stick_y));
        gamepadManager.getGamepad1().getRightBumper().setReleasedCommand(this::grabSequenceWithCheck);
//        Slides.INSTANCE.setPowerWithinLimits(-gamepad1.right_stick_y);

//        gamepadManager.getGamepad1().getRightBumper().setPressedCommand(newRootOpMode::grabSpecimen);
//        gamepadManager.getGamepad1().getDpadRight().setPressedCommand(newRootOpMode::scoreSpecimen);
//        gamepadManager.getGamepad1().getDpadRight().setReleasedCommand(newRootOpMode::resetArm);
//        gamepadManager.getGamepad1().getDpadUp().setPressedCommand(newRootOpMode::fullArm);
//        gamepadManager.getGamepad1().getDpadDown().setPressedCommand(newRootOpMode::resetArm);
//        gamepadManager.getGamepad1().getDpadLeft().setPressedCommand(Arm::reset);

//        touchingSubmersible().setPressedCommand(newRootOpMode::scoreSpecimen);
    }
}