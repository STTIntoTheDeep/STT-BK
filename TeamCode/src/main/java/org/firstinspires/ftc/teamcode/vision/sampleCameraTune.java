package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.opModes.rootOpMode;

@Config
@TeleOp(name = "Camera Tune",group = "TeleOp")
public class sampleCameraTune extends rootOpMode {
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    boolean intakeOpen = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(false);
        TeleOp = true;
        hardware.reduceHardwareCalls = false;
        hardware.servos.wrist.setServo(hardware.servoPositions.wristSampleCamera);
        intake.setElbow(hardware.servoPositions.cameraDown.getDifferential());
        hardware.reduceHardwareCalls = true;

        while (!isStarted() && !isStopRequested()) {
            chooseAlliance();
            telemetry.update();
        }

        intake.slidePID(0);

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

//            if (currentGamepad.dpad_left) {
//                sampleCamera();
//                hardware.servos.wrist.setServo(hardware.servoPositions.wristSampleCamera);
//            }
//            else if (currentGamepad.dpad_right) wideCamera();

            simpleIntakeSequence(currentGamepad.y && !previousGamepad.y, false, -currentGamepad.right_stick_y);
            if (intakeState == intakeStates.IDLE || intakeState == intakeStates.DONE) {
                if (currentGamepad.b && !previousGamepad.b)
                    intake.setElbow(hardware.servoPositions.cameraDown.getDifferential());
                if (currentGamepad.x && !previousGamepad.x) {
                    intakeOpen ^= true;
                    hardware.servos.intake.setServo((intakeOpen) ? hardware.servoPositions.intakeRelease : hardware.servoPositions.intakeGrip);
                }
            }

            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) samplePipeline.desiredColor = SampleDetectionPipeline.YELLOW;
            if (currentGamepad.dpad_left && !previousGamepad.dpad_left) samplePipeline.desiredColor = SampleDetectionPipeline.RED;
            if (currentGamepad.dpad_right && !previousGamepad.dpad_right) samplePipeline.desiredColor = SampleDetectionPipeline.BLUE;

            telemetry.addData("state",intakeState.ordinal());
            telemetry.addData("saveRAM", samplePipeline.saveRAM);
            if (samplePipeline.desiredColor == SampleDetectionPipeline.YELLOW) telemetry.addLine("yellow");
            else if (samplePipeline.desiredColor == SampleDetectionPipeline.BLUE) telemetry.addLine("blue");
            else if (samplePipeline.desiredColor == SampleDetectionPipeline.RED) telemetry.addLine("red");
            telemetry.addData("slideTarget", slideTarget);
            telemetry.addData("pidReady", intake.PIDReady());
            telemetry.addData("intakePower", hardware.motors.intake.getLastPower());
            telemetry.update();
        }
    }
}
