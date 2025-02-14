package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;

@Autonomous(name = "0+1 spec",group = "Autonomous")
public class Specimen1 extends rootOpMode {
    boolean specimenMode = true, high, goDown = false, outtakeNextState = false;
    int state = 0;

    double driveTimer;

    @Override
    public void runOpMode() {
        hardware.reduceHardwareCalls = false;
        initialize(false);
        hardware.reduceHardwareCalls = true;
        specimenPaths();
        follower.setMaxPower(0.6);
        follower.followPath(path1, true);
        telemetry.setMsTransmissionInterval(5);

        while (!isStarted()) {
            chooseSample();
        }

        if (isStopRequested()) return;

        outtake.scoreSpecimen(true);
        driveTimer = System.currentTimeMillis();
        while (opModeIsActive()) {
            follower.update();

            switch (state) {
                case 0:
                    if (!follower.isBusy() || driveTimer + 2750 < System.currentTimeMillis()) {
                        outtakeNextState = true;
                        state++;
                    }
                    break;
                case 1:
                    if (outtake.specimenStates == Outtake.sequenceStates.DOWN) {
                        follower.followPath(path2);
                        state++;
                    }
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(path3, true);
                        state++;
                    }
                    break;
                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(path4, true);
                        state++;
                    }
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(path3, true);
                        state++;
                    }
                    break;
                case 5:
                if (!follower.isBusy()) {
                    follower.followPath(path8, true);
                    state++;
                }
                break;
        }
//            follower.telemetryDebug(telemetry);

            if (outtakeNextState) {
                outtake.smashDown(true);
                outtakeNextState = false;
            } else {outtake.smashDown(false);}

            telemetry.addData("outtake", outtake.specimenStates.ordinal());
            telemetry.addData("pos", hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition());
            telemetry.addData("state",state);
            telemetry.addData("leftFront", hardware.motors.leftFront.getLastPower());
            telemetry.addData("rightFront", hardware.motors.rightFront.getLastPower());
            telemetry.addData("leftBack", hardware.motors.leftBack.getLastPower());
            telemetry.addData("rightBack", hardware.motors.rightBack.getLastPower());
            telemetry.update();
        }
    }
}