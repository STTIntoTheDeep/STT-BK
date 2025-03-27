package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "Sensor: REV touch sensor", group = "Sensor")
public class touchSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        // get a reference to our touchSensor object.
        hardware.touchSensors.leftFrontBumper.initSensor(hardwareMap);
        hardware.touchSensors.rightFrontBumper.initSensor(hardwareMap);
        hardware.motors.leftFront.initMotor(hardwareMap);
        hardware.motors.rightFront.initMotor(hardwareMap);
        hardware.motors.leftBack.initMotor(hardwareMap);
        hardware.motors.rightBack.initMotor(hardwareMap);

        // wait for the start button to be pressed.
        waitForStart();

        // while the OpMode is active, loop and read whether the sensor is being pressed.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            hardware.motors.leftFront.setPower(gamepad1.left_trigger);
            hardware.motors.rightFront.setPower(gamepad1.left_trigger);
            hardware.motors.leftBack.setPower(gamepad1.left_trigger);
            hardware.motors.rightBack.setPower(gamepad1.left_trigger);
            telemetry.addData("left", hardware.touchSensors.leftFrontBumper.sensorPressed());
            telemetry.addData("right", hardware.touchSensors.rightFrontBumper.sensorPressed());
            telemetry.update();
        }
    }
}
