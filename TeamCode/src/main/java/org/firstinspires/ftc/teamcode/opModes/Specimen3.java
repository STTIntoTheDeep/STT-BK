//package org.firstinspires.ftc.teamcode.opModes;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//
//import org.firstinspires.ftc.teamcode.Outtake;
//import org.firstinspires.ftc.teamcode.hardware;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.Pose;
//
//@Disabled
//@Autonomous(name = "Don't spec",group = "Autonomous")
//public class Specimen3 extends rootOpMode {
//    boolean specimenMode = true, high, goDown = false, outtakeNextState = false;
//    int state = 0;
//
//    double driveTimer, intakeTimer, intakeTarget;
//
//    @Override
//    public void runOpMode() {
//        hardware.reduceHardwareCalls = false;
//        initialize(false);
//        hardware.reduceHardwareCalls = true;
//        specimenPaths();
//
//        follower.setMaxPower(0.6);
//        follower.followPath(path1, true);
//
//        while (!isStarted()) {
//            chooseSample();
//        }
//
//        if (isStopRequested()) return;
//
//        outtake.scoreSpecimen(true);
//        driveTimer = System.currentTimeMillis() + 2750;
//        while (opModeIsActive()) {
//            follower.update();
//
//            switch (state) {
//                case 0:// Drive to sub first time
//                case 5:// Drive to sub second time
//                    if (!follower.isBusy() || driveTimer < System.currentTimeMillis()) {
//                        follower.setPose(new Pose(41.5, follower.getPose().getY(), 0));
//                        outtakeNextState = true;
//                        state++;
//                    }
//                    break;
//                case 1:// Score first specimen
//                    if (outtake.specimenStates == Outtake.sequenceStates.DOWN) {
//                        follower.followPath(path2);
//                        driveTimer = System.currentTimeMillis() + 4000;
//                        state++;
//                    }
//                    break;
//                case 2:// Push first spike mark sample
//                    if (!follower.isBusy() || driveTimer < System.currentTimeMillis()) {
//                        follower.followPath(path3);
//                        driveTimer = System.currentTimeMillis() + 5000;
//                        state++;
//                    }
//                    break;
//                case 3:// Get to second specimen
//                    if (!follower.isBusy() || driveTimer < System.currentTimeMillis()) {
//                        outtakeNextState = true;
//                        driveTimer = System.currentTimeMillis() + 300;
//                        state++;
//                    }
//                    break;
//                case 4:// Grab second specimen
//                    if (driveTimer < System.currentTimeMillis()) {
//                        follower.followPath(path4);
//                        driveTimer = System.currentTimeMillis() + 4000;
//                        state++;
//                    }
//                    break;
//                case 6:// Score second specimen
//                    if (outtake.specimenStates == Outtake.sequenceStates.DOWN) {
//                        state++;
//                    }
//                    break;
//            }
////            follower.telemetryDebug(telemetry);
//
//            if (outtakeNextState) {
//                outtake.smashDown(true);
//                outtakeNextState = false;
//            } else {outtake.smashDown(false);}
//
//            telemetry.addData("outtake", outtake.specimenStates.ordinal());
//            telemetry.addData("outtakePos", hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition());
//            telemetry.addData("state",state);
//            telemetry.addData("intakeTarget", intakeTarget);
//            telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
//            telemetry.addData("leftFront", hardware.motors.leftFront.getLastPower());
//            telemetry.addData("rightFront", hardware.motors.rightFront.getLastPower());
//            telemetry.addData("leftBack", hardware.motors.leftBack.getLastPower());
//            telemetry.addData("rightBack", hardware.motors.rightBack.getLastPower());
//            telemetry.update();
//        }
//    }
//}