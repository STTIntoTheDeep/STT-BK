package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;
@Disabled
@TeleOp(name = "SoloDrive",group = "TeleOp")
public class SoloDrive extends rootOpMode {
    boolean high, goDown = false;
    int rumbleStates, ledStates;

    Drivetrain drivetrain;

    // By setting these values to new Gamepad(), they will default to all
    // boolean values as false and all float values as 0
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() {
        initialize(true);
        intakeOpen = MathFunctions.roughlyEquals(hardware.servos.intake.getLastPosition(), hardware.servoPositions.intakeRelease.getPosition());
        outtakeOpen = MathFunctions.roughlyEquals(hardware.servos.outtakeClaw.getLastPosition(), hardware.servoPositions.outtakeRelease.getPosition());

        drivetrain = new MecanumDrivetrain(hardwareMap);

        while (!isStarted()) {
            chooseSample();
        }

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.y && !previousGamepad.y) samplePipeline.desiredColor = SampleDetectionPipeline.YELLOW;
            if (currentGamepad.x && !previousGamepad.x) samplePipeline.desiredColor = allianceColor;

            //TODO: add rumble
            simpleIntakeSequence(currentGamepad.a && !previousGamepad.a, false, -currentGamepad.left_stick_y);

            if (intakeState == intakeStates.IDLE || intakeState == intakeStates.DONE) {
                if (currentGamepad.b && !previousGamepad.b) {
                    intakeOpen ^= true;
                    hardware.servos.intake.setServo((intakeOpen) ? hardware.servoPositions.intakeRelease : hardware.servoPositions.intakeGrip);
                }
            }

            Vector input = new Vector(new Point(-gamepad1.left_stick_y, -gamepad1.left_stick_x, Point.CARTESIAN));
            drivetrain.run(new Vector(0,0), new Vector (-gamepad1.right_stick_x,0), input,  0);

            if (samplePipeline.desiredColor == SampleDetectionPipeline.YELLOW) telemetry.addLine("yellow");
            else if (samplePipeline.desiredColor == SampleDetectionPipeline.BLUE) telemetry.addLine("blue");
            else if (samplePipeline.desiredColor == SampleDetectionPipeline.RED) telemetry.addLine("red");

            telemetry.addData("intake ordinal", intakeState.ordinal());
            telemetry.addData("right pos", hardware.motors.outtake.dcMotorEx.getCurrentPosition());
            telemetry.update();
        }
    }
}