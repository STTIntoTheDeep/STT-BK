package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;

@TeleOp(name = "SoloDrive",group = "TeleOp")
public class SoloDrive extends rootOpMode {
    Vector driveInput;
    boolean specimenMode = true, high, goDown = false;
    int rumbleStates, ledStates;
    double outtakeLeftPos, outtakeRightPos;

    // By setting these values to new Gamepad(), they will default to all
    // boolean values as false and all float values as 0
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() {
        initialize(true);
        while (!isStarted()) {
            chooseSample();
        }

        //TODO: you can probably remove this then
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.options && !previousGamepad.options) {
                specimenMode ^= true;
                //TODO: if outtake doing anything, stop
                changingMode = true;
            }
            if (changingMode) changeMode(specimenMode);


            if (currentGamepad.x && !previousGamepad.x) {
                if (intakeState == intakeStates.IDLE) {
                    sampleDetectionPipeline.desiredColor = allianceColor;
                    intakeState = intakeStates.RELEASE;
                } else if (intakeState == intakeStates.EXTEND) intakeState = intakeStates.FIND;
            }

            if (currentGamepad.a && !previousGamepad.a && !specimenMode) {
                if (intakeState == intakeStates.IDLE) {
                    sampleDetectionPipeline.desiredColor = SampleDetectionPipeline.YELLOW;
                    intakeState = intakeStates.RELEASE;
                } else if (intakeState == intakeStates.EXTEND) intakeState = intakeStates.FIND;
            }

            //TODO: add rumble
            if (intakeState == intakeStates.DONE && !specimenMode) transferState = transferStates.UP;

            // Not allowed to move the outtake while transferring
            // TODO: if buttons pressed, buttonMode true and state to UP (outtake)
            //TODO: maybe indentation prettier?
            if ((transferState == transferStates.IDLE || transferState == transferStates.DONE) && !changingMode) {
                outtakeLeftPos = hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition();
                outtakeRightPos = hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition();
                outtake.slidesWithinLimits(currentGamepad.right_trigger - currentGamepad.left_trigger, outtakeLeftPos, outtakeRightPos);

                // Controls the outtake
                if (!outtake.buttonMode) {
                    // Normally you'd want a rising edge detector here but the hardware wrapper already covers that.
                    // This code automatically moves the shoulder to the right position
                    if (specimenMode) {
                        if (outtakeLeftPos < Outtake.slidePositions.CLEARS_ROBOT.getPosition())
                            hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
                        else hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderForward);
                    } else {
                        if (outtakeLeftPos < Outtake.slidePositions.CLEARS_ROBOT.getPosition())
                            hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderTransfer);
                        else hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
                    }
                    // Outtake claw toggle
                    if (currentGamepad.right_bumper && !previousGamepad.right_bumper) hardware.servos.outtakeClaw.setServo(
                            (hardware.servos.outtakeClaw.getLastPosition() == hardware.servoPositions.outtakeGrip.getPosition())
                                    ? hardware.servoPositions.outtakeRelease : hardware.servoPositions.outtakeGrip
                    );
                } else {
                    // do the PID thingy
                    //TODO: add rumble
                }
            }
            if (!changingMode) {
                if (specimenMode) {
                    transferSpecimen();
                } else {
                    transferSample();
                }
            }

            intakeSequence( currentGamepad.y && !previousGamepad.y, -currentGamepad.right_stick_y);

            follower.setTeleOpMovementVectors(-currentGamepad.left_stick_y, -currentGamepad.left_stick_x, -currentGamepad.right_stick_x, true);
            follower.update();

            telemetry.addData("mode", specimenMode);
            telemetry.update();
        }
    }

    private void outtake() {
        // Not allowed to move the outtake while transferring or changing mode
        if (changingMode) return;
        if (!(transferState == transferStates.IDLE || transferState == transferStates.DONE)) return;

        if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
            outtake.buttonMode = true;
            high = true;
        }
        if (currentGamepad.dpad_left && !previousGamepad.dpad_left) {
            outtake.buttonMode = true;
            high = false;
        }
        if (currentGamepad.dpad_down && !previousGamepad.dpad_down) {
            outtake.buttonMode = true;
            goDown = true;
        }

        // Controls the outtake
        if (!outtake.buttonMode) {
            outtakeLeftPos = hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition();
            outtakeRightPos = hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition();
            outtake.slidesWithinLimits(currentGamepad.right_trigger - currentGamepad.left_trigger, outtakeLeftPos, outtakeRightPos);
            // Normally you'd want a rising edge detector here but the hardware wrapper already covers that.
            // This code automatically moves the shoulder to the right position
            if (specimenMode) {
                if (outtakeLeftPos < Outtake.slidePositions.CLEARS_ROBOT.getPosition())
                    hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
                else hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderForward);
            } else {
                if (outtakeLeftPos < Outtake.slidePositions.CLEARS_ROBOT.getPosition())
                    hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderTransfer);
                else hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
            }
            // Outtake claw toggle
            if (currentGamepad.right_bumper && !previousGamepad.right_bumper) hardware.servos.outtakeClaw.setServo(
                    (hardware.servos.outtakeClaw.getLastPosition() == hardware.servoPositions.outtakeGrip.getPosition())
                            ? hardware.servoPositions.outtakeRelease : hardware.servoPositions.outtakeGrip
            );
        } else {
            if (goDown) {
                outtake.slidePID(Outtake.slidePositions.DOWN.getPosition());
                if (outtake.PIDReady()) goDown = false;
            } else {
                if (specimenMode) outtake.scoreSpecimen(high);
                else outtake.scoreBasket(high);
                //TODO: add rumble
            }
        }
        if (specimenMode) {
            transferSpecimen();
        } else {
            transferSample();
        }
    }
}