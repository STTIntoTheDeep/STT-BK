package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;

@Autonomous(name = "Auton1",group = "Autonomous")
public class Auton1 extends rootOpMode {
    boolean specimenMode = true, high, goDown = false;

    @Override
    public void runOpMode() {
        initialize(false);
        while (!isStarted()) {
            chooseSample();
        }

        if (isStopRequested()) return;
        follower.followPath(path1, true);
        follower.followPath(path2, true);
        follower.followPath(path3, true);
        follower.followPath(path4, true);
        follower.followPath(path5, true);
        follower.followPath(path6, true);
        follower.followPath(path7, true);
        follower.followPath(path8, true);

        while (opModeIsActive()) {
            follower.update();

            telemetry.addData("mode", specimenMode);
            telemetry.update();
        }
    }
}