package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomFilteredPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.KalmanFilterParameters;

/**
 * This is the FollowerConstants class. It holds many constants and parameters for various parts of
 * the Follower. This is here to allow for easier tuning of Pedro Pathing, as well as concentrate
 * everything tunable for the Paths themselves in one place.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @version 1.0, 3/4/2024
 */
@Config
public class FollowerConstants {
    //TODO: maybe remove this and just put it in TUNING.md, like the localizer.
    public enum Drivetrains {
        Mecanum,
        DifferentialSwerve
    }

    public static Drivetrains drivetrain = Drivetrains.Mecanum;

    public static int numberOfMotors = 4;

    //TODO transfer this to the enumerator hardware.java if that works

    // This section is for configuring your motors.
    public static String leftFrontMotorName = "left_front"; //leftFrontTop
    public static String leftRearMotorName = "left_back"; //leftFrontBottom
    public static String rightFrontMotorName = "right_front"; //rightFrontTop
    public static String rightRearMotorName = "right_back"; //rightFrontBottom
    //only necessary for six or eight motor drivetrains
    public static String backLeftFrontMotorName = "motor_five"; //leftBackTop
    public static String backLeftRearMotorName = "motor_six";//leftBackBottom
    public static String backRightFrontMotorName = "motor_seven"; //rightBackTop
    public static String backRightRearMotorName = "motor_eight";//rightBackBottom

    // For mecanum, these directions should be in such a way that when all powers are positive, the robot drives forwards.
    // For differential swerve, these directions should be in such a way that when all powers are positive, each pod drives forwards and doesn't rotate.
    public static DcMotorSimple.Direction leftFrontMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction rightRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    //only necessary for six or eight motor drivetrains
    public static DcMotorSimple.Direction backLeftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction backLeftRearMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction backRightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction backRightRearMotorDirection = DcMotorSimple.Direction.FORWARD;

    // This section is for setting the actual drive vector for the front left wheel, if the robot
    // is facing a heading of 0 radians with the wheel centered at (0,0).
    // Only necessary for mecanum drivetrains.
    private static final double xMovement = 68.2855596288241;
    private static final double yMovement = 56.54709353844553;
    private static final double[] convertToPolar = MathFunctions.cartesianToPolar(xMovement, -yMovement);
    public static Vector frontLeftVector = MathFunctions.normalizeVector(new Vector(convertToPolar[0],convertToPolar[1]));


    // Translational PIDF coefficients (don't use integral)
    public static CustomPIDFCoefficients translationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.15,
            0,
            0,
            0);

    // Translational Integral
    public static CustomPIDFCoefficients translationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);

    // Feed forward constant added on to the translational PIDF
    public static double translationalPIDFFeedForward = 0.04;


    // Heading error PIDF coefficients
    public static CustomPIDFCoefficients headingPIDFCoefficients = new CustomPIDFCoefficients(
            2,
            0,
            0.001,
            0);

    // Feed forward constant added on to the heading PIDF
    public static double headingPIDFFeedForward = 0.0;


    // Drive PIDF coefficients
    public static CustomFilteredPIDFCoefficients drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(
            0.004,
            0,
            0.0001,
            0.6,
            0);

    // Feed forward constant added on to the drive PIDF
    public static double drivePIDFFeedForward = 0.01;

    // Kalman filter parameters for the drive error Kalman filter
    public static KalmanFilterParameters driveKalmanFilterParameters = new KalmanFilterParameters(
            8,
            1);


    // Mass of robot in kilograms
    public static double mass = 12.5;

    // Centripetal force to power scaling
    public static double centripetalScaling = 0.0009;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    // Tested at 13.8V
    public static double forwardZeroPowerAcceleration = -410.0858991816445;

    // Acceleration of the drivetrain when power is cut in inches/second^2 (should be negative)
    // if not negative, then the robot thinks that its going to go faster under 0 power
    //Tested at 13.4V
    public static double lateralZeroPowerAcceleration = -857.6562290653185;

    // A multiplier for the zero power acceleration to change the speed the robot decelerates at
    // the end of paths.
    // Increasing this will cause the robot to try to decelerate faster, at the risk of overshoots
    // or localization slippage.
    // Decreasing this will cause the deceleration at the end of the Path to be slower, making the
    // robot slower but reducing risk of end-of-path overshoots or localization slippage.
    // This can be set individually for each Path, but this is the default.
    public static double zeroPowerAccelerationMultiplier = 0.05;


    // When the robot is at the end of its current Path or PathChain and the velocity goes below
    // this value, then end the Path. This is in inches/second.
    // This can be custom set for each Path.
    public static double pathEndVelocityConstraint = 0.1;

    // When the robot is at the end of its current Path or PathChain and the translational error
    // goes below this value, then end the Path. This is in inches.
    // This can be custom set for each Path.
    public static double pathEndTranslationalConstraint = 0.1;

    // When the robot is at the end of its current Path or PathChain and the heading error goes
    // below this value, then end the Path. This is in radians.
    // This can be custom set for each Path.
    public static double pathEndHeadingConstraint = 0.007;

    // When the t-value of the closest point to the robot on the Path is greater than this value,
    // then the Path is considered at its end.
    // This can be custom set for each Path.
    public static double pathEndTValueConstraint = 0.995;

    // When the Path is considered at its end parametrically, then the Follower has this many
    // milliseconds to further correct by default.
    // This can be custom set for each Path.
    public static double pathEndTimeoutConstraint = 500;

    // This is how many steps the BezierCurve class uses to approximate the length of a BezierCurve.
    public static int APPROXIMATION_STEPS = 1000;

    // This is scales the translational error correction power when the Follower is holding a Point.
    public static double holdPointTranslationalScaling = 0.45;

    // This is scales the heading error correction power when the Follower is holding a Point.
    public static double holdPointHeadingScaling = 0.35;

    // This is the number of times the velocity is recorded for averaging when approximating a first
    // and second derivative for on the fly centripetal correction. The velocity is calculated using
    // half of this number of samples, and the acceleration uses all of this number of samples.
    public static int AVERAGED_VELOCITY_SAMPLE_NUMBER = 8;

    // This is the number of steps the binary search for closest point uses. More steps is more
    // accuracy, and this increases at an exponential rate. However, more steps also does take more
    // time.
    public static int BEZIER_CURVE_BINARY_STEP_LIMIT = 10;


    // These activate / deactivate the secondary PIDs. These take over at errors under a set limit for
    // the translational, heading, and drive PIDs.
    public static boolean useSecondaryTranslationalPID = false;
    public static boolean useSecondaryHeadingPID = false;
    public static boolean useSecondaryDrivePID = false;


    // the limit at which the translational PIDF switches between the main and secondary translational PIDFs,
    // if the secondary PID is active
    public static double translationalPIDFSwitch = 3;

    // Secondary translational PIDF coefficients (don't use integral)
    public static CustomPIDFCoefficients secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(
            0.3,
            0,
            0.01,
            0);

    // Secondary translational Integral value
    public static CustomPIDFCoefficients secondaryTranslationalIntegral = new CustomPIDFCoefficients(
            0,
            0,
            0,
            0);

    // Feed forward constant added on to the small translational PIDF
    public static double secondaryTranslationalPIDFFeedForward = 0.015;


    // the limit at which the heading PIDF switches between the main and secondary heading PIDFs
    public static double headingPIDFSwitch = Math.PI / 20;

    // Secondary heading error PIDF coefficients
    public static CustomPIDFCoefficients secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(
            5,
            0,
            0.08,
            0);

    // Feed forward constant added on to the secondary heading PIDF
    public static double secondaryHeadingPIDFFeedForward = 0.01;


    // the limit at which the heading PIDF switches between the main and secondary drive PIDFs
    public static double drivePIDFSwitch = 20;

    // Secondary drive PIDF coefficients
    public static CustomFilteredPIDFCoefficients secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(
            0.02,
            0,
            0.000005,
            0.6,
            0);

    // Feed forward constant added on to the secondary drive PIDF
    public static double secondaryDrivePIDFFeedForward = 0.01;
}
