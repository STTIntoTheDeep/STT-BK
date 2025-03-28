package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware;
@Disabled
@TeleOp(name = "disagree",group = "Tests")
public class wrist extends rootOpMode {

    @Override
    public void runOpMode() {
        hardware.servos.wrist.initServo(hardwareMap);

        while (!isStarted() && !isStopRequested()) {
            hardware.servos.wrist.setServo(gamepad1.left_trigger);

            telemetry.addData("pos", gamepad1.left_trigger);
            telemetry.update();
        }
    }
}
