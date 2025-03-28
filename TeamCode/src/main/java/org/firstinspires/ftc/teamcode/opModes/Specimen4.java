//package org.firstinspires.ftc.teamcode.opModes;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.Outtake;
//import org.firstinspires.ftc.teamcode.hardware;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
//import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
//import org.firstinspires.ftc.teamcode.pedroPathing.util.Pose;
//
//@Autonomous(name = "DON'T USE 0+3 spec",group = "Autonomous")
//public class Specimen4 extends rootOpMode {
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
//        PathChain path10 = follower.pathBuilder().addPath(// Retrieve second specimen
//                        new BezierLine(
//                                new Point(40.7, 68.5),
//                                new Point(6, 36)))
//                .setConstantHeadingInterpolation(Math.toRadians(0))
//                .build();
//
//        PathChain path11 = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Point(6, 36),
//                                new Point(41, 75)
//                        )
//                )
//                .setConstantHeadingInterpolation(0)
//                .build();
//
//        PathChain path12 = follower.pathBuilder().addPath(
//                        new BezierCurve(
//                                new Point(41, 75),
//                                new Point(10, 25)
//                        )
//                )
//                .setConstantHeadingInterpolation(0)
//                .build();
//
//        follower.setMaxPower(0.6);
//        follower.followPath(path1, true);
//
//        while (!isStarted()) {
//            chooseAlliance();
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
//                case 4:// Drive to sub second time
//                    if (!follower.isBusy() || driveTimer < System.currentTimeMillis()) {
//                        outtakeNextState = true;
//                        state++;
//                    }
//                    break;
//                case 1:// Score first specimen
//                    if (outtake.specimenStates == Outtake.sequenceStates.DOWN) {
//                        follower.followPath(path10);
//                        driveTimer = System.currentTimeMillis() + 3000;
//                        state++;
//                    }
//                    break;
//                case 2:// Get second specimen
//                    if (!follower.isBusy() || driveTimer < System.currentTimeMillis()) {
//                        outtakeNextState = true;
//                        driveTimer = System.currentTimeMillis() + 300;
//                        state++;
//                    }
//                    break;
//                case 3:// Grab second specimen
//                    if (driveTimer < System.currentTimeMillis()) {
//                        follower.followPath(path11);
//                        driveTimer = System.currentTimeMillis() + 2750;
//                        state++;
//                    }
//                    break;
//                case 5:
//                    if (outtake.specimenStates == Outtake.sequenceStates.DOWN) {
//                        follower.followPath(path12);
//                        driveTimer = System.currentTimeMillis() + 5000;
//                        state++;
//                    }
//                    break;
//            }
//
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