package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;

import org.firstinspires.ftc.teamcode.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Slides;

@Config
@TeleOp(name = "Command Drive")
public class CommandDrive extends newRootOpMode {

    public static int position = 200;

    @Override
    public void onInit() {
        initOpMode(true);
        OuttakeClaw.INSTANCE.open().invoke();
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled.invoke();

        gamepadManager.getGamepad1().getDpadLeft().setPressedCommand(OuttakeClaw.INSTANCE::outtakeToggle);
        gamepadManager.getGamepad1().getDpadUp().setPressedCommand(newRootOpMode::scoreSpecimen);
        gamepadManager.getGamepad1().getDpadUp().setReleasedCommand(newRootOpMode::resetArm);
        gamepadManager.getGamepad1().getLeftBumper().setPressedCommand(newRootOpMode::grabSpecimen);
        gamepadManager.getGamepad1().getLeftBumper().setReleasedCommand(newRootOpMode::fullArm);

        gamepadManager.getGamepad1().getA().setPressedCommand(this::dropSample);
        gamepadManager.getGamepad1().getB().setPressedCommand(this::retractIntake);
        gamepadManager.getGamepad1().getX().setPressedCommand(IntakeClaw.INSTANCE::intakeToggle);
        gamepadManager.getGamepad1().getY().setPressedCommand(Elbow.INSTANCE::toggle);
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