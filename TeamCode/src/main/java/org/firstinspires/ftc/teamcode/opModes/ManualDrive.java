package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;

@TeleOp(name = "ManualDrive",group = "TeleOp")
public class ManualDrive extends rootOpMode {
    Drivetrain drivetrain;
    boolean specimenMode = true;
    int rumbleStates, ledStates;

    // By setting these values to new Gamepad(), they will default to all
    // boolean values as false and all float values as 0
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() {
        intake = new Intake(hardwareMap, TeleOp);
        outtake = new Outtake(hardwareMap, TeleOp);
        drivetrain = new MecanumDrivetrain(hardwareMap);
        hardware.motors.hook.initMotor(hardwareMap);
        hardware.motors.hook.dcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        /*
            Sets the hubs to using BulkReads. Any read will read all non-I2C sensors from a hub at once.
         */
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /*
            This line makes the telemetry available for FTC Dashboard by ACME Robotics.
         */
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(25);

        hardware.reduceHardwareCalls = false;
        wideCamera();
        hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
        hardware.reduceHardwareCalls = true;

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
            }

            if (currentGamepad.a && !previousGamepad.a) {
                if (intakeState == intakeStates.IDLE && transferState == transferStates.IDLE) {
                    samplePipeline.desiredColor = SampleDetectionPipeline.YELLOW;
                    intakeState = intakeStates.DOWN;
                    samplePipeline.saveRAM = false;
                    sampleCamera();
                } else if (intakeState == intakeStates.FIND) {
                    intakeState = intakeStates.DOWN;
                    intake.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
                } else if (intakeState == intakeStates.DOWN) {
                    hardware.servos.intake.setServo(hardware.servoPositions.intakeGrip);
                    continueTime = System.currentTimeMillis() + 300;
                    intakeState = intakeStates.GRAB;
                } else if (intakeState == intakeStates.DONE) intakeState = intakeStates.IDLE;
            }

            switch (intakeState) {
                case IDLE:
                    if (!intake.PIDReady() && !(transferState == transferStates.DOWN || transferState == transferStates.TRANSFER)) intake.slidePID(0);
                    break;
                case FIND:
                    hardware.motors.intake.setPower(Math.max(-currentGamepad.right_stick_y, -0.5));
                    break;
                case MOVE:
                    intake.slideCM(slideTarget);
                    if (intake.PIDReady() && continueTime < System.currentTimeMillis()) {
                        hardware.servos.intake.setServo(hardware.servoPositions.intakeGrip);
                        continueTime = System.currentTimeMillis() + 300;
                        intakeState = intakeStates.GRAB;
                    }
                    break;
                case GRAB:
                    if (continueTime < System.currentTimeMillis()) {
                        intakeState = intakeStates.BACK;
                        continueTime = System.currentTimeMillis() + intake.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
                        hardware.servos.wrist.setServo(hardware.servoPositions.wristTransfer);
                    }
                    break;
                case BACK:
                    if (continueTime < System.currentTimeMillis()) {
                        intakeState = intakeStates.PRE_DONE;
                        continueTime = System.currentTimeMillis() + intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
                    }
                    break;
                case PRE_DONE:
                    intake.slidePID(0);
                    if (continueTime < System.currentTimeMillis() && hardware.motors.intake.dcMotorEx.getCurrentPosition() < 40) {
                        intakeState = intakeStates.DONE;
                    }
                    break;
            }

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

                        if (hardware.motors.intake.dcMotorEx.getCurrentPosition() < 20) hardware.motors.intake.setPower(backPower);
                        else intake.slidePID(0);

                        if (outtake.PIDReady() && hardware.motors.intake.dcMotorEx.getCurrentPosition() < 20) {
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

            if (transferState == transferStates.IDLE || transferState == transferStates.DONE) {
                TeleOpOuttake(currentGamepad.right_trigger - currentGamepad.left_trigger, currentGamepad.right_bumper && !previousGamepad.right_bumper);
            }

            Vector input = new Vector(new Point(-gamepad1.left_stick_y, -gamepad1.left_stick_x, Point.CARTESIAN));
            drivetrain.run(new Vector(0,0), new Vector (-gamepad1.right_stick_x,0), input,  0);

            telemetry.addData("mode", specimenMode);
            telemetry.update();
        }
    }
}