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

            if (currentGamepad.dpad_left) {
                sampleCamera();
                hardware.servos.wrist.setServo(hardware.servoPositions.wristSampleCamera);
            }
            else if (currentGamepad.dpad_right) wideCamera();

            simpleIntakeSequence(currentGamepad.a && !previousGamepad.a, false, -currentGamepad.left_stick_y);
//            if (currentGamepad.a && !previousGamepad.a) {
//                if (intakeState == intakeStates.IDLE) {
//                    samplePipeline.desiredColor = SampleDetectionPipeline.YELLOW;
//                    intakeState = intakeStates.FIND;
//                    samplePipeline.saveRAM = false;
//                    sampleCamera();
//                } else if (intakeState == intakeStates.FIND && bestSampleInformation != null) {
//                    samplePipeline.saveRAM = true;
//                    hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
//                    continueTime = 500 + System.currentTimeMillis() + intake.wristToAngle(Math.toRadians(bestSampleInformation[2]) + 0.5*Math.PI - Math.acos(bestSampleInformation[0]/intake.armLength));
//                    intake.setElbow(new double[] {hardware.servoPositions.elbowCentered.getDifferential()[0], 0.0});
//                    slideTarget = intake.getSlideLength() + bestSampleInformation[1] - intake.armLength*Math.sin(Math.acos(-bestSampleInformation[0]/intake.armLength));
//                    intakeState = intakeStates.DOWN;
//                } else if (intakeState == intakeStates.DOWN) {
//                    intake.elbowYDistance(bestSampleInformation[0]);
//                    intakeState = intakeStates.MOVE;
//                } else if (intakeState == intakeStates.MOVE) {
//                    hardware.servos.intake.setServo(hardware.servoPositions.intakeGrip);
//                    intakeState = intakeStates.GRAB;
//                } else if (intakeState == intakeStates.GRAB) {
//                    intakeState = intakeStates.BACK;
//                    intake.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
//                    hardware.servos.wrist.setServo(hardware.servoPositions.wristTransfer);
//                } else if (intakeState == intakeStates.BACK) {
//                    intakeState = intakeStates.PRE_DONE;
//                    intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
//                }
//            }
//
//            switch (intakeState) {
//                case IDLE:
//                    if (!intake.PIDReady()) intake.slidePID(0);
//                    break;
//                case FIND:
//                    hardware.motors.intake.setPower(Math.max(-currentGamepad.left_stick_y, -0.5));
//                    chooseSample();
//                    break;
//                case DOWN:
//                    intake.slideCM(slideTarget);
//                    if (continueTime < System.currentTimeMillis()) {
//                        continueTime = System.currentTimeMillis() + intake.elbowYDistance(bestSampleInformation[0]) + 250;
//                        intakeState = intakeStates.MOVE;
//                    }
//                    break;
//                case MOVE:
//                    intake.slideCM(slideTarget);
//                    if (intake.PIDReady() && continueTime < System.currentTimeMillis()) {
//                        hardware.servos.intake.setServo(hardware.servoPositions.intakeGrip);
//                        continueTime = System.currentTimeMillis() + 300;
//                        intakeState = intakeStates.GRAB;
//                    }
//                    break;
//                case GRAB:
//                    if (continueTime < System.currentTimeMillis()) {
//                        intakeState = intakeStates.BACK;
//                        continueTime = System.currentTimeMillis() + intake.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
//                        hardware.servos.wrist.setServo(hardware.servoPositions.wristTransfer);
//                    }
//                    break;
//                case BACK:
//                    if (continueTime < System.currentTimeMillis()) {
//                        intakeState = intakeStates.PRE_DONE;
//                        continueTime = System.currentTimeMillis() + intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
//                    }
//                    break;
//                case PRE_DONE:
//                    intake.slidePID(0);
//                    if (continueTime < System.currentTimeMillis() && intake.PIDReady()) intakeState = intakeStates.DONE;
//                    break;
//            }

//            if (currentGamepad.y && !previousGamepad.y) samplePipeline.saveRAM ^= true;

            if (currentGamepad.y && !previousGamepad.y) samplePipeline.desiredColor = SampleDetectionPipeline.YELLOW;
            if (currentGamepad.x && !previousGamepad.x) samplePipeline.desiredColor = SampleDetectionPipeline.RED;
            if (currentGamepad.b && !previousGamepad.b) samplePipeline.desiredColor = SampleDetectionPipeline.BLUE;

            telemetry.addData("state",intakeState.ordinal());
            telemetry.addData("saveRAM", samplePipeline.saveRAM);
            if (samplePipeline.desiredColor == SampleDetectionPipeline.YELLOW) telemetry.addLine("yellow");
            else if (samplePipeline.desiredColor == SampleDetectionPipeline.BLUE) telemetry.addLine("blue");
            else if (samplePipeline.desiredColor == SampleDetectionPipeline.RED) telemetry.addLine("red");
            telemetry.addData("slideTarget", slideTarget);
            telemetry.addData("pidReady", intake.PIDReady());
            telemetry.update();
        }
    }
}
