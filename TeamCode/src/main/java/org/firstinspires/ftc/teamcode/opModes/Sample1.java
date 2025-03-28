package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Sample1",group = "Autonomous")
public class Sample1 extends rootOpMode {
    boolean specimenMode = true, high, goDown = false, outtakeNextState;
    int state = 0;

    double driveTimer;

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

        outtake.scoreSpecimen(true);
        driveTimer = System.currentTimeMillis() + 2750;
        while (opModeIsActive()) {
            follower.update();
            switch (state) {
                case 0:// Drive to sub second time
                    if (!follower.isBusy() || driveTimer < System.currentTimeMillis()) {
                        outtakeNextState = true;
                        state++;
                    }
                    break;
            }

//            if (outtakeNextState) {
//                outtake.scoreBasket(true, true);
//                outtakeNextState = false;
//            } else {outtake.scoreBasket(true, false);}

            follower.telemetryDebug(telemetry);

            telemetry.addData("mode", specimenMode);
            telemetry.update();
        }
    }
}