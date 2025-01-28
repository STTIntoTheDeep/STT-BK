package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "intakeTest",group = "Tests")
public class intakeTest extends rootOpMode {
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() {
        initialize(true);
        hardware.motors.intake.dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.intake.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

//            hardware.servos.elbowLeft.setServo(currentGamepad.left_trigger);
//            hardware.servos.elbowRight.setServo(currentGamepad.right_trigger);
//            hardware.servos.wrist.setServo(-currentGamepad.left_stick_y);
//            hardware.servos.intake.setServo(-currentGamepad.right_stick_y);
            hardware.motors.intake.setPower(-currentGamepad.left_stick_y);

            telemetry.addData("left trigger", currentGamepad.left_trigger);
            telemetry.addData("right trigger", currentGamepad.right_trigger);
            telemetry.addData("left stick", -currentGamepad.left_stick_y);
            telemetry.addData("right stick", -currentGamepad.right_stick_y);
            telemetry.update();
        }
    }
}
