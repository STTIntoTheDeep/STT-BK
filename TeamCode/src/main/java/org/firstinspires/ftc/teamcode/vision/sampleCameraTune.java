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

    double slideTarget;
    boolean intakeClaw = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initialize(false);
        hardware.reduceHardwareCalls = false;
        hardware.servos.wrist.setServo(hardware.servoPositions.wristSampleCamera);
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

            if (currentGamepad.a && !previousGamepad.a) {
                if (intakeState == intakeStates.IDLE) {
                    samplePipeline.desiredColor = SampleDetectionPipeline.YELLOW;
                    intakeState = intakeStates.FIND;
                    samplePipeline.saveRAM = false;
                    sampleCamera();
                } else if (intakeState == intakeStates.FIND && bestSampleInformation != null) {
                    hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
                    samplePipeline.saveRAM = true;
                    intake.wristToAngle(Math.toRadians(bestSampleInformation[2])+Math.acos(bestSampleInformation[0]/intake.armLength));
                    intake.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
                    slideTarget = intake.getSlideLength() + bestSampleInformation[1] - intake.armLength*Math.sin(Math.acos(-bestSampleInformation[0]/intake.armLength));
                    intakeState = intakeStates.DOWN;
                } else if (intakeState == intakeStates.DOWN) {
                    intakeTimer = System.currentTimeMillis();
                    intake.elbowYDistance(bestSampleInformation[1]);
                    intakeState = intakeStates.MOVE;
                } else if (intakeState == intakeStates.MOVE) {
                    hardware.servos.intake.setServo(hardware.servoPositions.intakeGrip);
                    intakeState = intakeStates.GRAB;
                } else if (intakeState == intakeStates.GRAB) {
                    intakeState = intakeStates.PRE_DONE;
                    intake.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
                } else if (intakeState == intakeStates.PRE_DONE) {
                    intakeState = intakeStates.DONE;
                    intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
                }
            }

            switch (intakeState) {
                case IDLE:
                    if (!intake.PIDReady()) intake.slidePID(0);
                    break;
                case EXTEND:
                    break;
                case FIND:
                    hardware.motors.intake.setPower(-currentGamepad.left_stick_y);
                    chooseSample();
                    break;
                case DOWN:
                    intake.slideCM(slideTarget);
                    break;
                case DONE:
                    intake.slidePID(0);
                    break;
            }

            if (currentGamepad.dpad_down && !previousGamepad.dpad_down) intakeClaw ^= true;
            hardware.servos.intake.setServo((intakeClaw) ? hardware.servoPositions.intakeGrip : hardware.servoPositions.intakeRelease);

//            if (currentGamepad.y && !previousGamepad.y) samplePipeline.saveRAM ^= true;
//
//            if (currentGamepad.a && !previousGamepad.a) samplePipeline.desiredColor = SampleDetectionPipeline.YELLOW;
//            if (currentGamepad.x && !previousGamepad.x) samplePipeline.desiredColor = SampleDetectionPipeline.RED;
//            if (currentGamepad.b && !previousGamepad.b) samplePipeline.desiredColor = SampleDetectionPipeline.BLUE;

            telemetry.addData("state",intakeState.ordinal());
            telemetry.addData("saveRAM", samplePipeline.saveRAM);
            if (samplePipeline.desiredColor == SampleDetectionPipeline.YELLOW) telemetry.addLine("yellow");
            else if (samplePipeline.desiredColor == SampleDetectionPipeline.BLUE) telemetry.addLine("blue");
            else if (samplePipeline.desiredColor == SampleDetectionPipeline.RED) telemetry.addLine("red");
            telemetry.addData("slideTarget", slideTarget);
            telemetry.update();
        }
    }
}
