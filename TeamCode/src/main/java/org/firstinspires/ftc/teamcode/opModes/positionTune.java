package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;

@Config
@TeleOp(name = "positionTune",group = "Tests")
public class positionTune extends rootOpMode {
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    boolean intakeOpen = true, outtakeOpen = true;
    public static int intakeTarget = 0, outtakeTarget = 0;
    public static double pitch = 0.18, yaw = 0.51, wristAngle = 0.14, shoulderPosition = 0.0,
            cameraX = hardware.cameraXPos, cameraY = hardware.cameraYPos, cameraZ = hardware.cameraZPos;
    //pitch -0.135 is down, 0.35 is transfer, 0.14 is camera 0, 0.245 is camera 45 degree
    // yaw 0.56 is parallel, 0.31 is right, 0.82 is left

    @Override
    public void runOpMode() {
        initialize(false);
        TeleOp = true;
        specimenMode = false;
        hardware.reduceHardwareCalls = false;
        hardware.servos.wrist.setServo(hardware.servoPositions.wristSampleCamera);
        intake.setElbow(hardware.servoPositions.cameraDown.getDifferential());
        hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
        hardware.reduceHardwareCalls = true;

        while (!isStarted() && !isStopRequested()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            hardware.cameraXPos = cameraX;
            hardware.cameraYPos = cameraY;
            hardware.cameraZPos = cameraZ;
            chooseSample();
            intake.setElbow(yaw, pitch);
            hardware.servos.wrist.setServo(wristAngle);
            if (currentGamepad.a && !previousGamepad.a) {
                intakeOpen ^= true;
                hardware.servos.intake.setServo((intakeOpen) ? hardware.servoPositions.intakeRelease : hardware.servoPositions.intakeGrip);
            }
            if (currentGamepad.b && !previousGamepad.b) {
                outtakeOpen ^= true;
                hardware.servos.outtakeClaw.setServo((outtakeOpen) ? hardware.servoPositions.outtakeRelease : hardware.servoPositions.outtakeGrip);
            }

            hardware.servos.shoulder.setServo(shoulderPosition);

            intake.slidePID(intakeTarget);
            outtake.slidePID(outtakeTarget);
            telemetry.update();
        }

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            chooseSample();
            intake.setElbow(yaw, pitch);
            hardware.servos.wrist.setServo(wristAngle);
            if (currentGamepad.a && !previousGamepad.a) {
                intakeOpen ^= true;
                hardware.servos.intake.setServo((intakeOpen) ? hardware.servoPositions.intakeRelease : hardware.servoPositions.intakeGrip);
            }
            if (currentGamepad.b && !previousGamepad.b) {
                outtakeOpen ^= true;
                hardware.servos.outtakeClaw.setServo((outtakeOpen) ? hardware.servoPositions.outtakeRelease : hardware.servoPositions.outtakeGrip);
            }

            hardware.servos.shoulder.setServo(shoulderPosition);

            intake.slidePID(intakeTarget);
            outtake.slidePID(outtakeTarget);

            telemetry.addData("left pos", hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition());
            telemetry.addData("right pos", hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition());
            telemetry.addData("outtake ready", outtake.PIDReady());
            telemetry.addData("intake ready", intake.PIDReady());
            telemetry.addData("intake motor pos", hardware.motors.intake.dcMotorEx.getCurrentPosition());
            telemetry.update();
        }
    }
}
