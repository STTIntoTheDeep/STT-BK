package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;

import java.util.List;

/**
 * This opmode isn't meant to be run directly, but instead be used as a template for TeleOp OpModes and to show some cool things you can do.
 * Most of this code is from <a href="https://gm0.org/en/latest/docs/software/tutorials/gamepad.html">Game Manual Zero</a>.
 * However, it also includes BulkReads, which you should always use to improve your loop times.
 * Additionally, it shows the best way to run our drive code.
 */
@Disabled
@TeleOp(name = "DemoTeleOp",group = "TeleOp")
public class DemoTeleOp extends LinearOpMode {
    // The constructors also initialize the hardware parts.
    Drivetrain drive = new MecanumDrivetrain(hardwareMap);

    double[] driveVector;
    boolean toggle;
    int rumbleStates, ledStates;

    // By setting these values to new Gamepad(), they will default to all
    // boolean values as false and all float values as 0
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();


    @Override
    public void runOpMode() {
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

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            /*
                Store the gamepad values from the previous loop iteration in
                previousGamepad1/2 to be used in this loop iteration.
                This is equivalent to doing this at the end of the previous
                loop iteration, as it will run in the same order except for
                the first/last iteration of the loop.
             */
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);

            /*
                Store the gamepad values from this loop iteration in
                currentGamepad1/2 to be used for the entirety of this loop iteration.
                This prevents the gamepad values from changing between being
                used and stored in previousGamepad1/2.
            */
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            /*
                The following code is a rising edge detector, which you can use to run code once, as soon as a button is pressed.
             */
            if (currentGamepad1.a && !previousGamepad1.a) {
                telemetry.addLine("Rising Edge Detected");
                //Run code, like setting a servo to a position. You can delete the telemetry.
            }



            /*
                The following code is a falling edge detector, which you can use to run code once, when a button is released.
                One button can run different code on the rising and falling edge.
            */
            if (!currentGamepad1.b && previousGamepad1.b) {
                telemetry.addLine("Falling Edge Detected");
                //Run code, like setting a servo to a position. You can delete the telemetry.
            }



            /*
                The following code is a toggle. When a button is pressed,
             */
            // Notice this is a rising edge detector.
            if (currentGamepad1.a && !previousGamepad1.a) {
                // This will set intakeToggle to true if it was previously false
                // and intakeToggle to false if it was previously true,
                // providing a toggling behavior.
                toggle = !toggle;
            }
            // Using the toggle variable to control the robot.
            if (toggle) {
                telemetry.addLine("Doing something");
            }
            else {
                telemetry.addLine("Doing the other thing.");
            }


            /*
                The following code gives feedback to the driver via the rumble function.
             */
            if (rumbleStates == 0) {
                //The simplest way to command rumble: rumble motor 1 at 100% power for a specified duration:
                gamepad1.rumble(1000);
            } else if (rumbleStates == 1) {
                //If control over both rumble motors and rumble intensity is desired:
                gamepad1.rumble(0.5, 0.7, 5000);
            } else if (rumbleStates == 2) {
                //To make a gamepad rumble for a certain number of “blips” (the notion of what a “blip” is being predefined by the SDK):
                gamepad1.rumbleBlips(10);
            } else if (rumbleStates == 3) {
                //To make the rumble stop. To check for rumble, use boolean isRumbling():
                gamepad1.stopRumble();
            } else if (rumbleStates == 4) {
                //To do fancy stuff:
                Gamepad.RumbleEffect effect = new Gamepad.RumbleEffect.Builder() //  You'd add this around line 36: Gamepad previousGamepad2 = new Gamepad();
                        .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                        .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                        .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                        .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
                        .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
                        .build();
                gamepad1.runRumbleEffect(effect);
            }



            /*
                The following code gives feedback to the driver via the LED bar on the controller (PS4 only).
            */
            if (ledStates == 0) {
                //To set the LED color for a fixed duration:
                gamepad1.setLedColor(1.0, 0.5, 1.0, 1000);
            } else if (ledStates == 1) {
                //To do fancy stuff:
                Gamepad.LedEffect rgbEffect = new Gamepad.LedEffect.Builder() //  You'd add this around line 36: Gamepad previousGamepad2 = new Gamepad();
                        .addStep(1, 0, 0, 250) // Show red for 250ms
                        .addStep(0, 1, 0, 250) // Show green for 250ms
                        .addStep(0, 0, 1, 250) // Show blue for 250ms
                        .addStep(1, 1, 1, 250) // Show white for 250ms
                        .build();
                gamepad1.runLedEffect(rgbEffect);
            }



            //TODO: FSM/sequence



            //TODO: PID



            /*
                Use our most accurate drive method.
                First, make a polar vector based on the controller input.
                Then, exaggerate the length of the value, as otherwise sliding friction would inhibit much of the joystick from being used.
                Then, input these values into our drive function.
             */
            Vector input = new Vector(new Point(-gamepad1.left_stick_y, -gamepad1.left_stick_x));
            double rotate = -0.5 * gamepad1.right_stick_x;
            drive.run(new Vector(0,0), new Vector(rotate,0), input, 0);


            /*
                Update Telemetry. By default, telemetry is only refreshed on the phone every 250 ms.
                Any calls to telemetry.update() during this window will be saved and overwritten if update() is called again before 250 ms have elapsed.
                setMsTransmissionInterval() may be used to change the amount of time the SDK will wait between sending updates to the driver station.
             */
            telemetry.update();
        }
    }
}
