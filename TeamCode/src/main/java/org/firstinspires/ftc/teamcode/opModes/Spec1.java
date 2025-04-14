package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;

import org.firstinspires.ftc.teamcode.subsystems.FollowPath;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeClaw;

@Autonomous(name = "1+0",group = "Autonomous")
public class Spec1 extends newRootOpMode {

    @Override
    public void onInit() {
        initOpMode(false);
        OuttakeClaw.INSTANCE.close().invoke();
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
//                new ParallelGroup(
//                        new ParallelRaceGroup(new FollowPath(path1, follower),
//                                new WaitUntil(hardware::touchingSubmersible)
//                        ),scoreSpecimen()
//                ),
                driveToSubmersible(new FollowPath(path1, true, 1.0, follower)),
                new InstantCommand(() -> follower.breakFollowing()),
                fullArm()
        ).invoke();
    }
}