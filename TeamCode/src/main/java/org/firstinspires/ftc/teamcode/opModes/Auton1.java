package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auton1",group = "Autonomous")
public class Auton1 extends rootOpMode {
    boolean specimenMode = true, high, goDown = false;
    int state = 0;

    @Override
    public void runOpMode() {
        initialize(false);
        while (!isStarted()) {
            chooseSample();
        }

        if (isStopRequested()) return;
        while (opModeIsActive()) {
            switch (state) {
                case 0:
                    follower.followPath(path1, true);
                    follower.update();
                    if (!follower.isBusy()) {
                        state++;
                    }
                    break;
                case 1:
                    follower.followPath(path2, true);
                    follower.update();
                    if (!follower.isBusy()) {
                        state++;
                    }
                    break;
                case 2:
                    follower.followPath(path3, true);
                    follower.update();
                    if (!follower.isBusy()) {
                        state++;
                    }
                    break;
                case 3:
                    follower.followPath(path4, true);
                    follower.update();
                    if (!follower.isBusy()) {
                        state++;
                    }
                    break;
                case 4:
                    follower.followPath(path5, true);
                    follower.update();
                    if (!follower.isBusy()) {
                        state++;
                    }
                    break;
                case 5:
                    follower.followPath(path6, true);
                    follower.update();
                    if (!follower.isBusy()) {
                        state++;
                    }
                    break;
                case 6:
                    follower.followPath(path7, true);
                    follower.update();
                    if (!follower.isBusy()) {
                        state++;
                    }
                    break;
                case 7:
                    follower.followPath(path8, true);
                    follower.update();
                    if (!follower.isBusy()) {
                        state++;
                    }
                    break;
                }

            follower.update();
            follower.telemetryDebug(telemetry);

            telemetry.addData("mode", specimenMode);
            telemetry.update();
        }
    }
}