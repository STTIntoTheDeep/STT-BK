package org.firstinspires.ftc.teamcode.opModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.hardware;

@TeleOp(name = "Sensor: REV touch sensor", group = "Sensor")
public class touchSensorTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        hardware.initSensors(hardwareMap);
        hardware.motors.leftFront.initMotor(hardwareMap);
        hardware.motors.rightFront.initMotor(hardwareMap);
        hardware.motors.leftBack.initMotor(hardwareMap);
        hardware.motors.rightBack.initMotor(hardwareMap);

        VoltageSensor sensor = hardwareMap.voltageSensor.iterator().next();
        double voltage = sensor.getVoltage(), timer = System.currentTimeMillis();

        // wait for the start button to be pressed.
        waitForStart();

        while (opModeIsActive()) {
            if (timer + 1000 < System.currentTimeMillis()) {
                timer = System.currentTimeMillis();
                voltage = sensor.getVoltage();
            }
            hardware.motors.leftFront.setPower(gamepad1.left_trigger);
            hardware.motors.rightFront.setPower(gamepad1.left_trigger);
            hardware.motors.leftBack.setPower(gamepad1.left_trigger);
            hardware.motors.rightBack.setPower(gamepad1.left_trigger);
            telemetry.addData("left", hardware.touchSensors.leftFrontBumper.pressed());
            telemetry.addData("right", hardware.touchSensors.rightFrontBumper.pressed());
            telemetry.addData("armDown", hardware.touchSensors.armDown.pressed());
            telemetry.addData("armUp", hardware.touchSensors.armUp.pressed());
            telemetry.addData("leftBackBumper", hardware.touchSensors.leftBackBumper.pressed());
            telemetry.addData("rightBackBumper", hardware.touchSensors.rightBackBumper.pressed());
            telemetry.addData("clawGrab", hardware.touchSensors.clawGrab.pressed());
            telemetry.addData("clawScore", hardware.touchSensors.clawScore.pressed());
            telemetry.addData("voltage", voltage);
            telemetry.update();
        }
    }
}