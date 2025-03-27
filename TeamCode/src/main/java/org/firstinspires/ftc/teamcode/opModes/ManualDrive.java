package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;

import java.util.List;

@TeleOp(name = "ManualDrive",group = "TeleOp")
public class ManualDrive extends rootOpMode {
    Drivetrain drivetrain;
    boolean specimenMode = true, clawHorizontal;

    int rumbleStates, ledStates;

    // By setting these values to new Gamepad(), they will default to all
    // boolean values as false and all float values as 0
    Gamepad currentGamepad = new Gamepad();
    Gamepad previousGamepad = new Gamepad();

    @Override
    public void runOpMode() {
        TeleOp = true;
        intake = new Intake(hardwareMap, TeleOp);
        outtake = new Outtake(hardwareMap, TeleOp);
        drivetrain = new MecanumDrivetrain(hardwareMap);
        hardware.motors.hook.initMotor(hardwareMap);
        hardware.motors.hook.dcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        /*
            Sets the hubs to using BulkReads. Any read will read all non-I2C sensors from a hub at once.
         */
        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /*
            This line makes the telemetry available for FTC Dashboard by ACME Robotics.
         */
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.setMsTransmissionInterval(25);

        while (!isStarted()) {
            chooseSample();
        }

        //TODO: you can probably remove this then
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad);
            currentGamepad.copy(gamepad1);

            if (currentGamepad.options && !previousGamepad.options) {
                specimenMode ^= true;
            }

            if (currentGamepad.b && !previousGamepad.b) {
                clawHorizontal ^= true;
                hardware.servos.wrist.setServo((clawHorizontal) ? hardware.servoPositions.wristTransfer : hardware.servoPositions.wristSampleCamera);
            }

            manualIntakeSequence(currentGamepad.y && !previousGamepad.y, -currentGamepad.right_stick_y);

            TeleOpOuttake(currentGamepad.right_trigger - currentGamepad.left_trigger, currentGamepad.a && !previousGamepad.a);

            Vector input = new Vector(new Point(-gamepad1.left_stick_y, -gamepad1.left_stick_x, Point.CARTESIAN));
            drivetrain.run(new Vector(0,0), new Vector (-gamepad1.right_stick_x*37.0/intake.getRobotLength(),0), input,  0);

            telemetry.addData("rightPos", hardware.motors.outtake.dcMotorEx.getCurrentPosition());
            telemetry.addData("rightPower", hardware.motors.outtake.getLastPower());
            telemetry.addData("mode", specimenMode);
            telemetry.update();
        }
    }
}