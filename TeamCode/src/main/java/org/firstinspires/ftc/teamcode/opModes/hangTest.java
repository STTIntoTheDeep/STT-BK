package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;

@TeleOp(group = "Tests")
public class hangTest extends rootOpMode {
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();
    Drivetrain drivetrain;

    @Override
    public void runOpMode() {
        initialize(false);
        drivetrain = new MecanumDrivetrain(hardwareMap);
        hardware.motors.outtakeLeft.dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.outtakeLeft.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.motors.outtakeRight.dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.outtakeRight.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hardware.reduceHardwareCalls = false;
        intake.setElbow(hardware.servoPositions.elbowTransfer.getDifferential());
        hardware.servos.wrist.setServo(hardware.servoPositions.wristTransfer);
        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
        hardware.reduceHardwareCalls = true;

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            hardware.motors.hook.setPower(currentGamepad.left_trigger - currentGamepad.right_trigger);

            outtake.slidesWithinLimits(-currentGamepad.left_stick_y);

            drivetrain.run(new Vector(new Point(-currentGamepad.left_stick_y, currentGamepad.left_stick_x, Point.CARTESIAN)), currentGamepad.right_stick_x);

            telemetry.addData("left pos", hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition());
            telemetry.addData("right pos", hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition());
            telemetry.addData("state",outtake.specimenStates.ordinal());
            telemetry.addData("specMode", specimenMode);
            telemetry.addData("changing", changingMode);
            telemetry.update();
        }
    }
}