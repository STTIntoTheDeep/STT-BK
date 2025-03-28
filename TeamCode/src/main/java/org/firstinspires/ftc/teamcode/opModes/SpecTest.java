package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelDeadlineGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelRaceGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Follower;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Pose;
import org.firstinspires.ftc.teamcode.subsystems.FollowPath;

@Autonomous(name = "specTest",group = "Autonomous")
public class SpecTest extends newRootOpMode {

    @Override
    public void onInit() {
        initOpMode(false);
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
//                new ParallelGroup(
//                        new ParallelRaceGroup(new FollowPath(path1, follower),
//                                new WaitUntil(hardware::touchingSubmersible)
//                        ),scoreSpecimen()
//                ),
                new ParallelDeadlineGroup(
                        new WaitUntil(hardware::touchingSubmersible),
                        new FollowPath(path1, true, 1.0, follower)),
                new FollowPath(path2, false, 0.6, follower),
//                new FollowPath(path3, false, 0.6, follower),
//                new FollowPath(path4, false, 0.6, follower),
                new FollowPath(path5, false, 0.6, follower),
                new FollowPath(path6, follower),
                new FollowPath(path7, follower))
        .invoke();
    }
}