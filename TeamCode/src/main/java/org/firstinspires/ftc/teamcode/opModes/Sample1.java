package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Sample1",group = "Autonomous")
public class Sample1 extends rootOpMode {
    boolean specimenMode = true, high, goDown = false;
    int state = 0;

    @Override
    public void runOpMode() {
        initialize(false);
        samplePaths();
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
//                    if (!follower.isBusy()) {
//                        follower.followPath(path3, true);
//                        state++;
//                    }
                    break;
            }
            follower.telemetryDebug(telemetry);

            telemetry.addData("mode", specimenMode);
            telemetry.update();
        }
    }
}