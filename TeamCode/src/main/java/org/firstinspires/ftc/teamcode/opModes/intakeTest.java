package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware;

@Config
@TeleOp(name = "intakeTest",group = "Tests")
public class intakeTest extends rootOpMode {
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    boolean open = true;
    public static int target = 0;
    public static double cm = 0, pitch = 0.2, yaw = 0.53;
    //pitch -0.135 is down, 0.35 is transfer, 0.14 is camera 0, 0.245 is camera 45 degree
    // yaw 0.56 is parallel, 0.31 is right, 0.82 is left

    @Override
    public void runOpMode() {
        initialize(true);
        hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
        intake.setElbow(hardware.servoPositions.cameraDown.getDifferential());

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);
            intake.setElbow(yaw, pitch);
            intake.slideCM(cm);

//            if (currentGamepad.a) hardware.servos.wrist.setServo(hardware.servoPositions.wristTransfer);
//            else hardware.servos.wrist.setServo(currentGamepad.left_trigger);
//            hardware.servos.elbowLeft.setServo(yaw - pitch);
//            hardware.servos.elbowRight.setServo(yaw + pitch);
//            intake.elbowYDistance(cm);
            hardware.servos.wrist.setServo(currentGamepad.left_trigger);
//            intake.wristToAngle(Math.toRadians(target));
//            hardware.servos.intake.setServo(currentGamepad.right_trigger);
            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
                open ^= true;
                hardware.servos.intake.setServo((open) ? hardware.servoPositions.intakeRelease : hardware.servoPositions.intakeGrip);
            }
//            hardware.motors.intake.setPower(-currentGamepad.left_stick_y);
//            intake.slidePID(target);
//            intake.slideCM(cm);

            telemetry.addData("left trigger", currentGamepad.left_trigger);
            telemetry.addData("right trigger", currentGamepad.right_trigger);
            telemetry.addData("left stick", -currentGamepad.left_stick_y);
            telemetry.addData("right stick", -currentGamepad.right_stick_y);
            telemetry.addData("pos", hardware.motors.intake.dcMotorEx.getCurrentPosition());
            telemetry.addData("intake pos", hardware.servos.intake.getLastPosition());
            telemetry.addData("target",target);
            telemetry.addData("power", hardware.motors.intake.getLastPower());
            telemetry.addData("wrist pos", hardware.servos.wrist.getLastPosition());
            telemetry.update();
        }
    }
}
