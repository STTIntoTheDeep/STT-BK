package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.conditionals.BlockingConditionalCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.Delay;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.subsystems.Elbow;
import org.firstinspires.ftc.teamcode.subsystems.FollowPath;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeClaw;

@Autonomous(name = "specTest",group = "Autonomous")
public class SpecTest extends newRootOpMode {
    long timer;

    @Override
    public void onInit() {
        initOpMode(false);
        hardware.reduceHardwareCalls = false;
        OuttakeClaw.INSTANCE.open().invoke();
        OuttakeClaw.INSTANCE.close().invoke();
        Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential());
        Elbow.INSTANCE.toTransfer().invoke();
        hardware.reduceHardwareCalls = true;
    }

    public Command cycleAnyways() {
        return new SequentialGroup(
                OuttakeClaw.INSTANCE.close(),
                driveToSubmersible(new FollowPath(path6, true, 1.0, follower)),
                new InstantCommand(() -> follower.breakFollowing()),
                scoreSpecimen(),
                new ParallelGroup(
                        driveToPickup(new FollowPath(path7, false, 1.0, follower)),
                        new Delay(0.5).then(resetArm())
                )
        );
    }

    public Command cycle() {
        return new BlockingConditionalCommand(
                () -> timer + 27000 > System.currentTimeMillis(),
                this::cycleAnyways,
                () -> Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential())
        );
    }

    @Override
    public void onStartButtonPressed() {
        new SequentialGroup(
                new InstantCommand(() -> timer = System.currentTimeMillis()),
//                new ParallelGroup(
//                        new ParallelRaceGroup(new FollowPath(path1, follower),
//                                new WaitUntil(hardware::touchingSubmersible)
//                        ),scoreSpecimen()
//                ),
                driveToSubmersible(new FollowPath(path1, true, 1.0, follower)),
                new InstantCommand(() -> follower.breakFollowing()),
                scoreSpecimen(),
                new ParallelGroup(
                        new FollowPath(path2, false, 1.0, follower),
                        new Delay(0.5).then(resetArm())
                ),
                driveToPickup(new FollowPath(path5, false, 1.0, follower)),
                cycleAnyways(),
                cycleAnyways(),
                cycleAnyways(),
                cycle()
        ).and(Elbow.INSTANCE.setElbow(hardware.servoPositions.elbowCentered.getDifferential()).afterTime(29)).invoke();
    }
}