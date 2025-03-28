//package org.firstinspires.ftc.teamcode.opModes;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.teamcode.Intake;
//import org.firstinspires.ftc.teamcode.Outtake;
//import org.firstinspires.ftc.teamcode.hardware;
//import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
//import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.MecanumDrivetrain;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;
//
//import java.util.List;
//
//@TeleOp(name = "Duodrive",group = "TeleOp")
//public class DuoDrive extends rootOpMode {
//    Drivetrain drivetrain;
//    boolean specimenMode = true, clawHorizontal = false, elbowDown = false;
//    int rumbleStates, ledStates;
//
//    // By setting these values to new Gamepad(), they will default to all
//    // boolean values as false and all float values as 0
//    Gamepad currentGamepad = new Gamepad(),
//            previousGamepad = new Gamepad(),
//    currentGamepad2 = new Gamepad(),
//    previousGamepad2 = new Gamepad();
//
//    @Override
//    public void runOpMode() {
//        intake = new Intake(hardwareMap, TeleOp);
//        outtake = new Outtake(hardwareMap, TeleOp);
//        drivetrain = new MecanumDrivetrain(hardwareMap);
//        hardware.motors.hook.initMotor(hardwareMap);
//        hardware.motors.hook.dcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
//
//        /*
//            Sets the hubs to using BulkReads. Any read will read all non-I2C sensors from a hub at once.
//         */
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//        /*
//            This line makes the telemetry available for FTC Dashboard by ACME Robotics.
//         */
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        telemetry.setMsTransmissionInterval(25);
//
//        hardware.reduceHardwareCalls = false;
//        intake.setElbow(0.51, 0.34);
//        hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
//        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
//        hardware.reduceHardwareCalls = true;
//
//        while (!isStarted()) {
//            chooseSample();
//        }
//
//        //TODO: you can probably remove this then
//        waitForStart();
//
//        if (isStopRequested()) return;
//
//        while (opModeIsActive()) {
//            previousGamepad.copy(currentGamepad);
//            currentGamepad.copy(gamepad1);
//
//            if (currentGamepad.options && !previousGamepad.options) {
//                specimenMode ^= true;
//            }
//
//            if (currentGamepad.b && !previousGamepad.b) {
//                clawHorizontal ^= true;
//                hardware.servos.wrist.setServo((clawHorizontal) ? hardware.servoPositions.wristTransfer : hardware.servoPositions.wristSampleCamera);
//            }
//
//            if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
//                elbowDown ^= true;
//            }
//            if (elbowDown) {
//                intake.elbowYDistance(intake.armLength * currentGamepad2.left_stick_x);
//            } else {
//                intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
//            }
//            hardware.servos.wrist.setServo(new Vector(new Point(currentGamepad2.right_stick_x, -currentGamepad2.right_stick_y)).getTheta());
//
//            if (currentGamepad.a && !previousGamepad.a) {
//                intakeOpen ^= true;
//                hardware.servos.intake.setServo((intakeOpen) ? hardware.servoPositions.intakeRelease : hardware.servoPositions.intakeGrip);
//            }
//            intake.slideWithinLimits(currentGamepad2.left_trigger - currentGamepad2.right_trigger);
//
//            if (!specimenMode) {
//                switch (transferState) {
//                    case IDLE:
//                        if (intakeState == intakeStates.GRAB) transferState = transferStates.UP;
//                        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
//                        transferTimer = System.currentTimeMillis() + 250;
//                        break;
//                    case UP:
//                        outtake.slidePID(Outtake.slidePositions.CLEARS_INTAKE.getPosition());
//                        if (transferTimer < System.currentTimeMillis() && outtake.PIDReady()) {
//                            hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderTransfer);
//                            transferTimer = System.currentTimeMillis() + 250;
//                            transferState = transferStates.DOWN;
//                        }
//                        break;
//                    case DOWN:
//                        if (transferTimer < System.currentTimeMillis() && intakeState == intakeStates.DONE) {
//                            outtake.slidePID(Outtake.slidePositions.TRANSFER.getPosition());
//
//                            if (hardware.motors.intake.dcMotorEx.getCurrentPosition() < 20)
//                                hardware.motors.intake.setPower(backPower);
//                            else intake.slidePID(0);
//
//                            if (outtake.PIDReady() && hardware.motors.intake.dcMotorEx.getCurrentPosition() < 20) {
//                                hardware.motors.intake.setPower(backPower);
//                                transferTimer = System.currentTimeMillis() + 400;
//                                outtake.slidePID(Outtake.slidePositions.TRANSFER.getPosition());
//                                hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeGrip);
//                                hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
//                                transferState = transferStates.TRANSFER;
//                            }
//                        } else outtake.slidePID(Outtake.slidePositions.CLEARS_INTAKE.getPosition());
//                        break;
//                    case TRANSFER:
//                        hardware.motors.intake.setPower(backPower);
//                        outtake.slidePID(Outtake.slidePositions.TRANSFER.getPosition());
//                        if (transferTimer < System.currentTimeMillis() && outtake.PIDReady()) {
//                            transferState = transferStates.UP_AGAIN;
//                            intakeState = intakeStates.IDLE;
//                        }
//                        break;
//                    case UP_AGAIN:
//                        outtake.slidePID(Outtake.slidePositions.CLEARS_INTAKE.getPosition());
//                        if (outtake.PIDReady()) {
//                            transferState = transferStates.BACK;
//                            hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
//                            transferTimer = 300 + System.currentTimeMillis();
//                        }
//                        break;
//                    case BACK:
//                        outtake.slidePID(Outtake.slidePositions.CLEARS_INTAKE.getPosition());
//                        if (transferTimer < System.currentTimeMillis()) {
//                            transferState = transferStates.DOWN_AGAIN;
//                        }
//                        break;
//                    case DOWN_AGAIN:
//                        outtake.slidePID(Outtake.slidePositions.DOWN.getPosition());
//                        if (outtake.PIDReady()) {
//                            transferState = transferStates.DONE;
//                        }
//                        break;
//                    case DONE:
//                        //FIXME: this only runs once
//                        outtake.slidePID(Outtake.slidePositions.DOWN.getPosition());
//                        transferState = transferStates.IDLE;
//                        break;
//                }
//            }
//
//            if (transferState == transferStates.IDLE || transferState == transferStates.DONE) {
//                TeleOpOuttake(currentGamepad.right_trigger - currentGamepad.left_trigger, currentGamepad.a && !previousGamepad.a);
//            }
//
//            Vector input = new Vector(new Point(-gamepad1.left_stick_y, -gamepad1.left_stick_x, Point.CARTESIAN));
//            drivetrain.run(new Vector(0,0), new Vector (-gamepad1.right_stick_x*37.0/intake.getRobotLength(),0), input,  0);
//
//            telemetry.addData("leftPos", hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition());
//            telemetry.addData("rightPos", hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition());
//            telemetry.addData("leftPower", hardware.motors.outtakeLeft.getLastPower());
//            telemetry.addData("rightPower", hardware.motors.outtakeRight.getLastPower());
//            telemetry.addData("mode", specimenMode);
//            telemetry.update();
//        }
//    }
//}