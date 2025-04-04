package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.ftc.driving.MecanumDriverControlled;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeClaw;

@Config
@TeleOp(name = "Command Drive")
public class CommandDrive extends newRootOpMode {

    public static int position = 200;

    @Override
    public void onInit() {
        initOpMode(false);
        OuttakeClaw.INSTANCE.open().invoke();
    }

    @Override
    public void onStartButtonPressed() {
        MotorEx[] motors = new MotorEx[] {
                new MotorEx(hardware.motors.leftFront.dcMotorEx),
                new MotorEx(hardware.motors.rightFront.dcMotorEx),
                new MotorEx(hardware.motors.leftBack.dcMotorEx),
                new MotorEx(hardware.motors.rightBack.dcMotorEx)
        };
        driverControlled = new MecanumDriverControlled(motors, gamepadManager.getGamepad1());
        driverControlled.invoke();

        gamepadManager.getGamepad1().getRightBumper().setPressedCommand(newRootOpMode::grabSpecimen);
        gamepadManager.getGamepad1().getDpadRight().setPressedCommand(newRootOpMode::scoreSpecimen);
        gamepadManager.getGamepad1().getDpadRight().setReleasedCommand(newRootOpMode::resetArm);
        gamepadManager.getGamepad1().getDpadUp().setPressedCommand(newRootOpMode::fullArm);
        gamepadManager.getGamepad1().getDpadDown().setPressedCommand(newRootOpMode::resetArm);
        gamepadManager.getGamepad1().getDpadLeft().setPressedCommand(Arm::reset);
//        touchingSubmersible().setPressedCommand(newRootOpMode::scoreSpecimen);
    }
}