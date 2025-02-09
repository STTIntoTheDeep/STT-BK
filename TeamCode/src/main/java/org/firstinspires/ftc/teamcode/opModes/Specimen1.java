package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.hardware;

@Autonomous(name = "0+0 spec",group = "Autonomous")
public class Specimen1 extends rootOpMode {
    boolean specimenMode = true, high, goDown = false;
    int state = 0;

    @Override
    public void runOpMode() {
        initialize(false);
        specimenPaths();
        follower.setMaxPower(0.6);
        follower.followPath(path1, true);

        while (!isStarted()) {
            chooseSample();
        }

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            follower.update();
            switch (state) {
                case 0:
                    if (!follower.isBusy()) {
                        follower.followPath(path2, true);
                        state++;
                    }
                    break;
                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(path3, true);
                        state++;
                    }
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(path4, true);
                        state++;
                    }
                    break;
                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(path5, true);
                        state++;
                    }
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(path6, true);
                        state++;
                    }
                    break;
                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(path7, true);
                        state++;
                    }
                    break;
                case 6:
                    if (!follower.isBusy()) {
                        follower.followPath(path8, true);
                        state++;
                    }
                    break;
                case 7:
                    if (!follower.isBusy()) {
                        state++;
                    }
                    break;
                }
            follower.telemetryDebug(telemetry);

            telemetry.addData("mode", specimenMode);
            telemetry.addData("leftFront", hardware.motors.leftFront.getLastPower());
            telemetry.addData("rightFront", hardware.motors.rightFront.getLastPower());
            telemetry.addData("leftBack", hardware.motors.leftBack.getLastPower());
            telemetry.addData("rightBack", hardware.motors.rightBack.getLastPower());
            telemetry.update();
        }
    }
}