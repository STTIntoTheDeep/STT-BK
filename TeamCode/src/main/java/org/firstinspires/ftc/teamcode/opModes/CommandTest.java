package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.subsystems.FollowPath;

@Config
@TeleOp(name = "Command Test")
public class CommandTest extends newRootOpMode {

    public static int position = 200;

//    public Command routine() {
//        return new PerpetualCommand(new SequentialGroup(
//                Arm.INSTANCE.toPosition(position)
////                new InstantCommand(() -> OpModeData.telemetry.addData("pos", hardware.motors.outtake.dcMotorEx.getCurrentPosition()))
//        ));
//    }

//    public Command routine() {
//        return new SequentialGroup(
//                OuttakeClaw.INSTANCE.open(),
//                new Delay(1.0),
//                new ParallelGroup(
//                        OuttakeClaw.INSTANCE.close(),
//                        IntakeClaw.INSTANCE.close()
//                )
//        );
//    }

    @Override
    public void onInit() {
        initOpMode(false);
    }

    @Override
    public void onStartButtonPressed() {
        new FollowPath(move, follower).invoke();
    }
}