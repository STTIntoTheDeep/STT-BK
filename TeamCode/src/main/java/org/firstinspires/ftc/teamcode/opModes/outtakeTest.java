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

    boolean specimenMode = false;

    @Override
    public void runOpMode() {
        initialize(true);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            hardware.motors.outtakeLeft.setPower(-currentGamepad.left_stick_y);
            hardware.motors.outtakeRight.setPower(-currentGamepad.right_stick_y);

            //TODO: proper servo positions
//            hardware.servos.shoulder.setServo(currentGamepad.right_trigger);

            //TODO: working encoder
//            if (-currentGamepad.left_stick_y == 0 && -currentGamepad.right_stick_y == 0) {
//                outtake.slidesWithinLimits(-gamepad2.left_stick_y, hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition(), hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition());
//            }

            //TODO: tune PID
//            outtake.slidePID(target);

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
            if (currentGamepad.a && !previousGamepad.a) sampleStates++;


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

            //TODO: check if changeMode works
            if (currentGamepad.options && !previousGamepad.options) {
                specimenMode ^= true;
                //TODO: if outtake doing anything, stop
                changingMode = true;
            }
            if (changingMode) changeMode(specimenMode);

            //TODO: add rumble
            //TODO: after this, test if MecanumDrivetrain works on it's own, and in Follower.java

            telemetry.addData("left stick", -currentGamepad.left_stick_y);
            telemetry.addData("right stick", -currentGamepad.right_stick_y);
            telemetry.addData("right trigger", currentGamepad.right_trigger);
            telemetry.addData("left pos", hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition());
            telemetry.addData("right pos", hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition());
            telemetry.addData("sampleState",sampleStates);
            telemetry.addData("specMode", specimenMode);
            telemetry.addData("changing", changingMode);
            telemetry.update();
        }
    }
}