package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware;

//TODO: remove @Config
@Config
@TeleOp(name = "outtakeTest",group = "Tests")
public class outtakeTest extends rootOpMode {
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    public static double target, power;

    @Override
    public void runOpMode() {
        initialize(false);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            hardware.motors.outtake.setPower(power);
            hardware.servos.outtakeClaw.setServo(target);

            telemetry.addData("pos", hardware.motors.outtake.dcMotorEx.getCurrentPosition());
            telemetry.addData("state",outtake.specimenStates.ordinal());
            telemetry.update();
        }
    }
}