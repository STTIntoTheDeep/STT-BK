package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelGroup;
import com.rowanmcalpin.nextftc.core.command.groups.SequentialGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.PerpetualCommand;
import com.rowanmcalpin.nextftc.ftc.NextFTCOpMode;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.IntakeClaw;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeClaw;

@Config
@TeleOp(name = "NextFTC Autonomous Program 2 Java")
public class NextFtcTest extends NextFTCOpMode {
    public NextFtcTest() {
        super(OuttakeClaw.INSTANCE, IntakeClaw.INSTANCE, Arm.INSTANCE);
    }

    public static int position = 200;

//    public Command routine() {
//        return new PerpetualCommand(new SequentialGroup(
//                Arm.INSTANCE.toPosition(position)
////                new InstantCommand(() -> OpModeData.telemetry.addData("pos", hardware.motors.outtake.dcMotorEx.getCurrentPosition()))
//        ));
//    }

    public Command routine() {
        return new SequentialGroup();
    }

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
    }

    @Override
    public void onStartButtonPressed() {
        routine().invoke();
    }
}