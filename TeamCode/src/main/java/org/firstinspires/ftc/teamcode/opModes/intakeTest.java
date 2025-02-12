package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;

@Config
@TeleOp(name = "intakeTest",group = "Tests")
public class intakeTest extends rootOpMode {
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    public static int target = 0;
    public static double cm = 0, pitch = 0.18, yaw = 0.51, angle, backPower = -0.25;
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
            chooseAlliance();
            telemetry.update();
        }

        if (isStopRequested()) return;

        intake.slidePID(0);

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.dpad_left) {
                sampleCamera();
                hardware.servos.wrist.setServo(hardware.servoPositions.wristSampleCamera);
            }
            else if (currentGamepad.dpad_right) wideCamera();

            manualIntakeSequence(currentGamepad.y && !previousGamepad.y, -currentGamepad.right_stick_y);

//            if (transferState == transferStates.IDLE && !specimenMode && currentGamepad.a && !previousGamepad.a) {
//                hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
//                transferTimer = System.currentTimeMillis() + 250;
//                transferState = transferStates.UP;
//            }
            transfer();
//            if (intakeState == intakeStates.DONE && !specimenMode) transferState = transferStates.UP;
            if (transferState != transferStates.IDLE) {
//                if (currentGamepad.a && !previousGamepad.a) {
//                    if (transferState == transferStates.UP) {
//                        telemetry.addData("pid ready", outtake.PIDReady());
//                        hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderTransfer);
//                        transferTimer = System.currentTimeMillis() + 250;
//                        transferState = transferStates.DOWN;
//                    } else if (transferState == transferStates.DOWN) {
//                        transferState = transferStates.TRANSFER;
//                        transferTimer = System.currentTimeMillis() + 300;
//                        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeGrip);
//                        hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
//                        telemetry.addData("pid ready", outtake.PIDReady());
//                    } else if (transferState == transferStates.TRANSFER) {
//                        transferState = transferStates.UP_AGAIN;
//                    } else if (transferState == transferStates.UP_AGAIN) {
//                        transferState = transferStates.BACK;
//                        hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
//                        transferTimer = 300 + System.currentTimeMillis();
//                    } else if (transferState == transferStates.BACK) {
//                        transferState = transferStates.DOWN_AGAIN;
//                    } else if (transferState == transferStates.DOWN_AGAIN) {
//                        transferState = transferStates.DONE;
//                    }
//                }

            } else {
                if (changingMode) {

                } else {
//                    TeleOpOuttake();
                }
            }

//            intake.slideCM(cm);

//            intake.setElbow(yaw, pitch);
//            intake.elbowYDistance(cm);

//            intake.wristToAngle(Math.toRadians(angle) + 0.5*Math.PI - Math.acos(cm/intake.armLength));
//            intake.wristToAngle(angle);

//            hardware.servos.intake.setServo(currentGamepad.right_trigger);
//            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
//                open ^= true;
//                hardware.servos.intake.setServo((open) ? hardware.servoPositions.intakeRelease : hardware.servoPositions.intakeGrip);
//            }

            telemetry.addData("intake ordinal", intakeState.ordinal());
            telemetry.addData("transfer ordinal", transferState.ordinal());
            telemetry.addData("left pos", hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition());
            telemetry.addData("right pos", hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition());
            telemetry.addData("outtake ready", outtake.PIDReady());
            telemetry.addData("intake ready", hardware.motors.intake.dcMotorEx.getCurrentPosition() < 40);

//            telemetry.addData("left trigger", currentGamepad.left_trigger);
//            telemetry.addData("right trigger", currentGamepad.right_trigger);
//            telemetry.addData("left stick", -currentGamepad.left_stick_y);
//            telemetry.addData("right stick", -currentGamepad.right_stick_y);
            telemetry.addData("intake motor pos", hardware.motors.intake.dcMotorEx.getCurrentPosition());
            telemetry.addData("intake pos", hardware.servos.intake.getLastPosition());
            telemetry.addData("target",target);
            telemetry.addData("power", hardware.motors.intake.getLastPower());
            telemetry.addData("wrist pos", hardware.servos.wrist.getLastPosition());
            telemetry.update();
        }
    }
}
