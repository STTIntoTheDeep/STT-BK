package org.firstinspires.ftc.teamcode.pedroPathing.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.PoseUpdater;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;

import java.util.ArrayList;

/**
 * This is the ForwardVelocityTuner autonomous tuning OpMode. This runs the robot forwards at max
 * power until it reaches some specified distance. It records the most recent velocities, and on
 * reaching the end of the distance, it averages them and prints out the velocity obtained. It is
 * recommended to run this multiple times on a full battery to get the best results. What this does
 * is, when paired with StrafeVelocityTuner, allows FollowerConstants to create a Vector that
 * empirically represents the direction your mecanum wheels actually prefer to go in, allowing for
 * more accurate following.
 * You can adjust the distance the robot will travel on FTC Dashboard: 192/168/43/1:8080/dash
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/13/2024
 */
//TODO: documentation
@Config
@Autonomous(name = "Forward Velocity Tuner", group = "Autonomous Pathing Tuning")
public class ForwardVelocityTuner extends OpMode {
    private final ArrayList<Double> velocities = new ArrayList<>();

    Drivetrain drivetrain;

    private PoseUpdater poseUpdater;

    public static double DISTANCE = 48;
    public static double RECORD_NUMBER = 10;

    private Telemetry telemetryA;

    private boolean end;

    /**
     * This initializes the drive motors as well as the cache of velocities and the FTC Dashboard
     * telemetry.
     */
    @Override
    public void init() {
        poseUpdater = new PoseUpdater(hardwareMap);
        drivetrain = new MecanumDrivetrain(hardwareMap);

        for (int i = 0; i < RECORD_NUMBER; i++) {
            velocities.add(0.0);
        }

        telemetryA = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetryA.addLine("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.");
        telemetryA.addLine("Make sure you have enough room, since the robot has inertia after cutting power.");
        telemetryA.addLine("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.");
        telemetryA.addLine("Press CROSS or A on game pad 1 to stop.");
        telemetryA.update();

    }

    /**
     * This starts the OpMode by setting the drive motors to run forward at full power.
     */
    @Override
    public void start() {
        drivetrain.run(new Vector(1,0),0);
        end = false;
    }

    /**
     * This runs the OpMode. At any point during the running of the OpMode, pressing CROSS or A on
     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
     * velocities, and when the robot has run forward enough, these last velocities recorded are
     * averaged and printed.
     */
    @Override
    public void loop() {
        if (gamepad1.cross || gamepad1.a) {
            drivetrain.stopMotors();
            hardware.motors.leftFront.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motors.leftBack.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motors.rightFront.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motors.rightBack.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            requestOpModeStop();
        }

        poseUpdater.update();
        if (!end) {
            if (Math.abs(poseUpdater.getPose().getX()) > DISTANCE) {
                end = true;
                drivetrain.stopMotors();
                hardware.motors.leftFront.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.motors.leftBack.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.motors.rightFront.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                hardware.motors.rightBack.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            } else {
                double currentVelocity = Math.abs(MathFunctions.dotProduct(poseUpdater.getVelocity(), new Vector(1, 0)));
                velocities.add(currentVelocity);
                velocities.remove(0);
            }
        } else {
            drivetrain.stopMotors();
            hardware.motors.leftFront.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motors.leftBack.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motors.rightFront.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hardware.motors.rightBack.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            double average = 0;
            for (Double velocity : velocities) {
                average += velocity;
            }
            average /= velocities.size();

            telemetryA.addData("forward velocity:", average);
            telemetryA.update();
        }
    }
}