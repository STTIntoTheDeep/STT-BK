package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.NullCommand;
import com.rowanmcalpin.nextftc.core.command.utility.conditionals.BlockingConditionalCommand;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.Slides;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@Config
@TeleOp(name = "Manual Drive")
public class CommandManual extends newRootOpMode {

    public static int position = 200;

    private Command elbowThingy() {
        return new BlockingConditionalCommand(
                () -> hardware.servos.elbowLeft.getLastPosition() != hardware.servoPositions.elbowTransfer.getDifferential()[0] - hardware.servoPositions.elbowTransfer.getDifferential()[1]
                && hardware.servos.elbowLeft.getLastPosition() != hardware.servoPositions.elbowDrop.getDifferential()[0] - hardware.servoPositions.elbowDrop.getDifferential()[1],
                () -> Elbow.INSTANCE.yDistance(gamepad1.left_stick_x),
                NullCommand::new
        );
    }

    @Override
    public void onInit() {
        initOpMode(true);
    }

    @Override
    public void onStartButtonPressed() {
        driverControlled.invoke();

        gamepadManager.getGamepad2().getDpadLeft().setPressedCommand(OuttakeClaw.INSTANCE::outtakeToggle);
        gamepadManager.getGamepad2().getDpadLeft().setHeldCommand(() -> Arm.INSTANCE.setPower(gamepad2.right_trigger - gamepad2.left_trigger));
        gamepadManager.getGamepad2().getDpadUp().setPressedCommand(Arm.INSTANCE::toHigh);
        gamepadManager.getGamepad2().getDpadRight().setReleasedCommand(Arm.INSTANCE::toDown);
        gamepadManager.getGamepad2().getLeftBumper().setPressedCommand(newRootOpMode::grabSpecimen);
        gamepadManager.getGamepad2().getLeftBumper().setReleasedCommand(newRootOpMode::fullArm);

        gamepadManager.getGamepad2().getA().setPressedCommand(this::dropSample);
        gamepadManager.getGamepad2().getB().setPressedCommand(this::retractIntake);
        gamepadManager.getGamepad2().getX().setPressedCommand(Wrist.INSTANCE::toggle);
        gamepadManager.getGamepad2().getY().setPressedCommand(Elbow.INSTANCE::toggle);
        gamepadManager.getGamepad2().getRightStick().getButton().setHeldCommand(() -> Slides.INSTANCE.setPowerWithinLimits(-gamepad2.right_stick_y));
        gamepadManager.getGamepad2().getRightBumper().setPressedCommand(IntakeClaw.INSTANCE::toggle);
        gamepadManager.getGamepad2().getLeftStick().getButton().setPressedCommand(this::elbowThingy);
    }
}