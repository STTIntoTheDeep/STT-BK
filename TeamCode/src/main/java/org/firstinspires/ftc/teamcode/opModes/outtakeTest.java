package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;

//TODO: remove @Config
@Config
@TeleOp(name = "outtakeTest",group = "Tests")
public class outtakeTest extends rootOpMode {
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    public static int target;

    int sampleStates = 0;

    double timer, outtakeRightPos, outtakeLeftPos;

    boolean specimenMode = true;

    @Override
    public void runOpMode() {
        initialize(true);
        hardware.motors.outtakeLeft.dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.outtakeLeft.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.motors.outtakeRight.dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.outtakeRight.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

//            hardware.motors.outtakeLeft.setPower(-currentGamepad.left_stick_y);
//            hardware.motors.outtakeRight.setPower(-currentGamepad.right_stick_y);

//            hardware.servos.outtakeClaw.setServo(currentGamepad.left_trigger);
//            hardware.servos.shoulder.setServo(currentGamepad.right_trigger);

//            if (-currentGamepad.left_stick_y == 0) outtake.slidePID(target);
//            outtake.slidesWithinLimits(-currentGamepad.left_stick_y);
            hardware.motors.hook.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
//            double power = -gamepad1.left_stick_y;
//            outtake.slidesWithinLimits(power, hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition(),hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition());

            //TODO: make sequence work
//            switch (sampleStates) {
//                case 0:
//                    break;
//                case 1:
//                    outtake.slidePID(target);
//                    break;
//                case 2:
//                    hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
//                    timer = System.currentTimeMillis();
//                    sampleStates++;
//                    break;
//                case 3:
//                    if (timer + 200 < System.currentTimeMillis()) {
//                        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
//                        timer = System.currentTimeMillis();
//                        sampleStates++;
//                    }
//                    break;
//                case 4:
//                    if (timer + 200 < System.currentTimeMillis()) sampleStates = 0;
//                    break;
//            }
//            if (currentGamepad.a && !previousGamepad.a) sampleStates++;


            //TODO: check this works
//            outtakeLeftPos = hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition();
//            outtakeRightPos = hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition();
//            outtake.slidesWithinLimits(currentGamepad.right_trigger - currentGamepad.left_trigger, outtakeLeftPos, outtakeRightPos);
//            if (specimenMode) {
//                if (outtakeLeftPos < Outtake.slidePositions.CLEARS_ROBOT.getPosition())
//                    hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
//                else hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderForward);
//            } else {
//                if (outtakeLeftPos < Outtake.slidePositions.CLEARS_ROBOT.getPosition())
//                    hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderTransfer);
//                else hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
//            }
//            if (currentGamepad.right_bumper && !previousGamepad.right_bumper) hardware.servos.outtakeClaw.setServo(
//                    (hardware.servos.outtakeClaw.getLastPosition() == hardware.servoPositions.outtakeGrip.getPosition())
//                            ? hardware.servoPositions.outtakeRelease : hardware.servoPositions.outtakeGrip
//            );
            outtake.scoreSpecimen(currentGamepad.a && !previousGamepad.a && !currentGamepad.options);
            //TODO: sequence + triggers
//            if (!outtake.buttonMode && currentGamepad.a && !previousGamepad.a && !currentGamepad.options) outtake.buttonMode = true;
//            if (outtake.buttonMode) {
//                outtake.scoreSpecimen((outtake.specimenStates != Outtake.sequenceStates.SCORED) ?
//                        currentGamepad.a && !previousGamepad.a && !currentGamepad.options :
//                        previousGamepad.a && !currentGamepad.a && !currentGamepad.options);
//            } else {
//                outtake.leftPos = hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition();
//                outtake.rightPos = hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition();
//                outtake.slidesWithinLimits(currentGamepad.right_trigger - currentGamepad.left_trigger);
//                if (specimenMode) {
//                    if (outtake.leftPos < Outtake.slidePositions.CLEARS_ROBOT.getPosition())
//                        hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
//                    else hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderForward);
//                } else {
//                    if (outtake.leftPos < Outtake.slidePositions.CLEARS_ROBOT.getPosition())
//                        hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderTransfer);
//                    else hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
//                }
//                // Outtake claw toggle
//                if (currentGamepad.right_bumper && !previousGamepad.right_bumper) hardware.servos.outtakeClaw.setServo(
//                        (hardware.servos.outtakeClaw.getLastPosition() == hardware.servoPositions.outtakeGrip.getPosition())
//                                ? hardware.servoPositions.outtakeRelease : hardware.servoPositions.outtakeGrip);
//            }
            //TODO: check if changeMode works
//            if (currentGamepad.options && !previousGamepad.options) {
//                specimenMode ^= true;
//                //TODO: if outtake doing anything, stop
//                changingMode = true;
//            }
//            if (changingMode) changeMode(specimenMode);

            //TODO: add rumble

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();
//            telemetry.addData("left stick", -currentGamepad.left_stick_y);
//            telemetry.addData("right stick", -currentGamepad.right_stick_y);
//            telemetry.addData("left trigger", currentGamepad.left_trigger);
//            telemetry.addData("right trigger", currentGamepad.right_trigger);
            telemetry.addData("left pos", hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition());
            telemetry.addData("right pos", hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition());
            telemetry.addData("state",outtake.specimenStates.ordinal());
            telemetry.addData("specMode", specimenMode);
            telemetry.addData("changing", changingMode);
            telemetry.update();
        }
    }
}