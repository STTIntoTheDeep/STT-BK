package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;

@TeleOp(name = "SoloDrive",group = "TeleOp")
public class SoloDrive extends rootOpMode {
    boolean high, goDown = false;
    int rumbleStates, ledStates;

    Drivetrain drivetrain;

    // By setting these values to new Gamepad(), they will default to all
    // boolean values as false and all float values as 0
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() {
        initialize(true);
        drivetrain = new MecanumDrivetrain(hardwareMap);
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


//            if (currentGamepad.x && !previousGamepad.x) {
//                if (intakeState == intakeStates.IDLE) {
//                    samplePipeline.desiredColor = allianceColor;
//                    intakeState = intakeStates.EXTEND;
//                } else if (intakeState == intakeStates.EXTEND) intakeState = intakeStates.FIND;
//            }
//
//            if (currentGamepad.a && !previousGamepad.a && !specimenMode) {
//                if (intakeState == intakeStates.IDLE) {
//                    samplePipeline.desiredColor = SampleDetectionPipeline.YELLOW;
//                    intakeState = intakeStates.EXTEND;
//                } else if (intakeState == intakeStates.EXTEND) intakeState = intakeStates.FIND;
//            }

            //TODO: add rumble
            if (intakeState == intakeStates.DONE && !specimenMode) transferState = transferStates.UP;

            if (!(transferState == transferStates.IDLE || transferState == transferStates.DONE)) {

            } else {
                if (changingMode) {

                } else {
                    TeleOpOuttake();
                }
            }
            simpleIntakeSequence(currentGamepad.a && !previousGamepad.a, false, -currentGamepad.left_stick_y);
            if (intakeState == intakeStates.IDLE || intakeState == intakeStates.DONE) {
                if (currentGamepad.b && !previousGamepad.b) {
                    intakeOpen ^= true;
                    hardware.servos.intake.setServo((intakeOpen) ? hardware.servoPositions.intakeRelease : hardware.servoPositions.intakeGrip);
                }
            }

            Vector input = new Vector(new Point(-gamepad1.left_stick_y, -gamepad1.left_stick_x, Point.CARTESIAN));
            drivetrain.run(new Vector(0,0), new Vector (-gamepad1.right_stick_x,0), input,  0);
            telemetry.addData("mode", specimenMode);
            telemetry.addData("slide pos", hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition());
            telemetry.update();
        }
    }

    private void TeleOpOuttake() {
        // Not allowed to move the outtake while transferring or changing mode
        // TODO: if buttons pressed, buttonMode true and state to UP (outtake)
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