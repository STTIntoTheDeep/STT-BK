package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;

@TeleOp(name = "ManualDrive",group = "TeleOp")
public class ManualDrive extends rootOpMode {
    boolean specimenMode = true, high, goDown = false;
    int rumbleStates, ledStates;

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

            //TODO: add rumble
            if (intakeState == intakeStates.DONE && !specimenMode) transferState = transferStates.UP;

            outtake();

            follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);
            follower.update();

            telemetry.addData("mode", specimenMode);
            telemetry.update();
        }
    }

    private void outtake() {
        // Not allowed to move the outtake while transferring or changing mode
        // TODO: if buttons pressed, buttonMode true and state to UP (outtake)
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
            outtake.leftPos = hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition();
            outtake.rightPos = hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition();
            outtake.slidesWithinLimits(currentGamepad.right_trigger - currentGamepad.left_trigger);
            // Normally you'd want a rising edge detector here but the hardware wrapper already covers that.
            // This code automatically moves the shoulder to the right position
            if (specimenMode) {
                if (outtake.leftPos < Outtake.slidePositions.CLEARS_ROBOT.getPosition())
                    hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
                else hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderForward);
            } else {
                if (outtake.leftPos < Outtake.slidePositions.CLEARS_ROBOT.getPosition())
                    hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderTransfer);
                else hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
            }
            // Outtake claw toggle
            if (currentGamepad.right_bumper && !previousGamepad.right_bumper) hardware.servos.outtakeClaw.setServo(
                    (hardware.servos.outtakeClaw.getLastPosition() == hardware.servoPositions.outtakeGrip.getPosition())
                            ? hardware.servoPositions.outtakeRelease : hardware.servoPositions.outtakeGrip);
        } else {
            if (goDown) {
                outtake.slidePID(Outtake.slidePositions.DOWN.getPosition());
                if (outtake.PIDReady()) goDown = false;
            } else {
//                if (specimenMode) outtake.scoreSpecimen(high);
//                else outtake.scoreBasket(high);
                //TODO: add rumble
            }
        }
        if (specimenMode) {
//            transferSpecimen();
        } else {
//            transferSample();
        }
    }
}