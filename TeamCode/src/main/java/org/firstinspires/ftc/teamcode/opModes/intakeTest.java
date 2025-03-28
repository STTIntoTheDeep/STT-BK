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

    public static int target = 0;
    public static double cm = 0, pitch = 0.18, yaw = 0.51, angle, claw, backPower = -0.25;
    //pitch -0.135 is down, 0.35 is transfer, 0.14 is camera 0, 0.245 is camera 45 degree
    // yaw 0.56 is parallel, 0.31 is right, 0.82 is left

    boolean open = false;

    @Override
    public void runOpMode() {
        initialize(false);
        TeleOp = true;
        hardware.reduceHardwareCalls = false;
        hardware.servos.wrist.setServo(hardware.servoPositions.wristSampleCamera);
        intake.setElbow(hardware.servoPositions.cameraDown.getDifferential());
        hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);
        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
        hardware.reduceHardwareCalls = true;

        while (!isStarted() && !isStopRequested()) {
            chooseAlliance();
            telemetry.update();
        }

        if (isStopRequested()) return;

        intake.slidePID(0);

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.dpad_left) {
                sampleCamera();
                hardware.servos.wrist.setServo(hardware.servoPositions.wristSampleCamera);
            }
            else if (currentGamepad.dpad_right) wideCamera();

//            simpleIntakeSequence(currentGamepad.y && !previousGamepad.y, currentGamepad.a, currentGamepad.right_trigger  -currentGamepad.left_trigger);

//            intake.slideCM(cm);

//            intake.setElbow(yaw, pitch);
//            intake.elbowYDistance(cm);

//            intake.wristToAngle(Math.toRadians(angle) + 0.5*Math.PI - Math.acos(cm/intake.armLength));
//            intake.wristToAngle(angle);
            hardware.servos.wrist.setServo(angle);

            hardware.servos.intake.setServo(claw);
//            if (currentGamepad.dpad_up && !previousGamepad.dpad_up) {
//                open ^= true;
//                hardware.servos.intake.setServo((open) ? hardware.servoPositions.intakeRelease : hardware.servoPositions.intakeGrip);
//            }

            telemetry.addData("intake ordinal", intakeState.ordinal());
            telemetry.addData("right pos", hardware.motors.outtake.dcMotorEx.getCurrentPosition());
            telemetry.addData("outtake ready", outtake.PIDReady());
            telemetry.addData("intake ready", hardware.motors.intake.dcMotorEx.getCurrentPosition() < 40);

            telemetry.addData("intake motor pos", hardware.motors.intake.dcMotorEx.getCurrentPosition());
            telemetry.addData("intake pos", hardware.servos.intake.getLastPosition());
            telemetry.addData("target",target);
            telemetry.addData("power", hardware.motors.intake.getLastPower());
            telemetry.addData("wrist pos", hardware.servos.wrist.getLastPosition());
            telemetry.update();
        }
    }
}
