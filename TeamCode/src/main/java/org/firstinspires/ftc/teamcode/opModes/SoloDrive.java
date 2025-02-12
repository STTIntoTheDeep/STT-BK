package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;

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
        intakeOpen = MathFunctions.roughlyEquals(hardware.servos.intake.getLastPosition(), hardware.servoPositions.intakeRelease.getPosition());
        outtakeOpen = MathFunctions.roughlyEquals(hardware.servos.outtakeClaw.getLastPosition(), hardware.servoPositions.outtakeRelease.getPosition());

        drivetrain = new MecanumDrivetrain(hardwareMap);

        while (!isStarted()) {
            chooseSample();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.options && !previousGamepad.options) {
                specimenMode ^= true;
            }

            if (currentGamepad.y && !previousGamepad.y) samplePipeline.desiredColor = SampleDetectionPipeline.YELLOW;
            if (currentGamepad.x && !previousGamepad.x) samplePipeline.desiredColor = allianceColor;

            //TODO: add rumble
            simpleIntakeSequence(currentGamepad.a && !previousGamepad.a, false, -currentGamepad.left_stick_y);

            switch (transferState) {
                case IDLE:
                    if (intakeState == intakeStates.GRAB) transferState = transferStates.UP;
                    hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
                    transferTimer = System.currentTimeMillis() + 250;
                    break;
                case UP:
                    outtake.slidePID(Outtake.slidePositions.CLEARS_INTAKE.getPosition());
                    if (transferTimer < System.currentTimeMillis() && outtake.PIDReady()) {
                        hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderTransfer);
                        transferTimer = System.currentTimeMillis() + 250;
                        transferState = transferStates.DOWN;
                    }
                    break;
                case DOWN:
                    if (transferTimer < System.currentTimeMillis() && intakeState == intakeStates.DONE) {
                        outtake.slidePID(Outtake.slidePositions.TRANSFER.getPosition());

                        intake.slidePID(0);

                        if (outtake.PIDReady() && intake.PIDReady() && transferTimer < System.currentTimeMillis()) {
                            hardware.motors.intake.setPower(backPower);
                            transferTimer = System.currentTimeMillis() + 400;
                            outtake.slidePID(Outtake.slidePositions.TRANSFER.getPosition());
                            hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeGrip);
                            hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
                            transferState = transferStates.TRANSFER;
                        }
                    } else outtake.slidePID(Outtake.slidePositions.CLEARS_INTAKE.getPosition());
                    break;
                case TRANSFER:
                    hardware.motors.intake.setPower(backPower);
                    outtake.slidePID(Outtake.slidePositions.TRANSFER.getPosition());
                    if (transferTimer < System.currentTimeMillis() && outtake.PIDReady()) {
                        transferState = transferStates.UP_AGAIN;
                        intakeState = intakeStates.IDLE;
                    }
                    break;
                case UP_AGAIN:
                    outtake.slidePID(Outtake.slidePositions.CLEARS_INTAKE.getPosition());
                    if (outtake.PIDReady()) {
                        transferState = transferStates.BACK;
                        hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
                        transferTimer = 300 + System.currentTimeMillis();
                    }
                    break;
                case BACK:
                    outtake.slidePID(Outtake.slidePositions.CLEARS_INTAKE.getPosition());
                    if (transferTimer < System.currentTimeMillis()) {
                        transferState = transferStates.DOWN_AGAIN;
                    }
                    break;
                case DOWN_AGAIN:
                    outtake.slidePID(Outtake.slidePositions.DOWN.getPosition());
                    if (outtake.PIDReady()) {
                        transferState = transferStates.DONE;
                    }
                    break;
                case DONE:
                    //FIXME: this only runs once
                    outtake.slidePID(Outtake.slidePositions.DOWN.getPosition());
                    transferState = transferStates.IDLE;
                    break;
            }

            if (transferState == transferStates.IDLE) {
                if (changingMode) {
                    //TODO: change mode
                } else {
                    TeleOpOuttake(currentGamepad.right_trigger - currentGamepad.left_trigger, currentGamepad.right_bumper && !previousGamepad.right_bumper);
                }
            }
            if (intakeState == intakeStates.IDLE || intakeState == intakeStates.DONE) {
                if (currentGamepad.b && !previousGamepad.b) {
                    intakeOpen ^= true;
                    hardware.servos.intake.setServo((intakeOpen) ? hardware.servoPositions.intakeRelease : hardware.servoPositions.intakeGrip);
                }
            }

            Vector input = new Vector(new Point(-gamepad1.left_stick_y, -gamepad1.left_stick_x, Point.CARTESIAN));
            drivetrain.run(new Vector(0,0), new Vector (-gamepad1.right_stick_x,0), input,  0);

            if (samplePipeline.desiredColor == SampleDetectionPipeline.YELLOW) telemetry.addLine("yellow");
            else if (samplePipeline.desiredColor == SampleDetectionPipeline.BLUE) telemetry.addLine("blue");
            else if (samplePipeline.desiredColor == SampleDetectionPipeline.RED) telemetry.addLine("red");

            telemetry.addData("mode", specimenMode);
            telemetry.addData("intake ordinal", intakeState.ordinal());
            telemetry.addData("transfer ordinal", transferState.ordinal());
            telemetry.addData("left pos", hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition());
            telemetry.addData("right pos", hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition());
            telemetry.update();
        }
    }
}