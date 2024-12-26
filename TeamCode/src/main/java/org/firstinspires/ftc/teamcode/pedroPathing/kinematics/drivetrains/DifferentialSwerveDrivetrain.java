package org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains;

import android.hardware.Sensor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

import java.util.Arrays;
import java.util.List;

/**
 * This is the TwoPodDifferentialSwerveDrivetrain class, based off Pedro-Pathings DriveVectorScaler class. This class takes in inputs Vectors for driving, heading
 * correction, and translational/centripetal correction is able to run the motors for a two-pod, three-pod or four-pod differential swerve drivetrain.
 *
 * @author Dean van Beek - 3977 Stanislas Tech Team
 * @version 1.0, 24/12/2024
 */
public class DifferentialSwerveDrivetrain extends Drivetrain {
    HardwareMap hardwareMap;

    //TODO: test if moving to initialize doesn't cause an NPE
    private DcMotorEx leftFrontTop, leftFrontBottom, rightFrontBottom, rightFrontTop, leftBackTop, leftBackBottom, rightBackTop, rightBackBottom;
    private DcMotorEx sensor1, sensor2, sensor3, sensor4;

    private Point centre;

    private List<DifferentialSwervePod> pods;

    DifferentialSwervePod
            leftFront,
            rightFront,
            leftRear,
            rightRear;

    /**
     * This creates a new TwoPodDifferentialSwerveDrivetrain, which takes in various movement vectors and outputs
     * the wheel drive powers necessary to move in the intended direction, given the true movement
     * vector for the front left mecanum wheel.
     */
    public DifferentialSwerveDrivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        initialize();
    }

    @Override
    public void initialize() {
        leftFrontTop = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        leftFrontBottom = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftRearMotorName);
        rightFrontTop = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        rightFrontBottom = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightRearMotorName);
        leftFrontTop.setDirection(FollowerConstants.leftFrontMotorDirection);
        leftFrontBottom.setDirection(FollowerConstants.leftRearMotorDirection);
        rightFrontTop.setDirection(FollowerConstants.rightFrontMotorDirection);
        rightFrontBottom.setDirection(FollowerConstants.rightRearMotorDirection);

        //only necessary for three or four pod drivetrains
//        leftBackTop = hardwareMap.get(DcMotorEx.class, FollowerConstants.backLeftFrontMotorName);
//        leftBackBottom = hardwareMap.get(DcMotorEx.class, FollowerConstants.backLeftFrontMotorName);
//        rightBackTop = hardwareMap.get(DcMotorEx.class, FollowerConstants.backRightFrontMotorName);
//        rightBackBottom = hardwareMap.get(DcMotorEx.class, FollowerConstants.backRightRearMotorName);
//        leftBackTop.setDirection(FollowerConstants.backLeftFrontMotorDirection);
//        leftBackBottom.setDirection(FollowerConstants.backLeftRearMotorDirection);
//        rightBackTop.setDirection(FollowerConstants.backRightFrontMotorDirection);
//        rightBackBottom.setDirection(FollowerConstants.backRightRearMotorDirection);

        motors = Arrays.asList(leftFrontTop, leftFrontBottom, rightFrontBottom, rightFrontTop);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        sensor1 = hardwareMap.get(DcMotorEx.class, FollowerConstants.leftFrontMotorName);
        sensor2 = hardwareMap.get(DcMotorEx.class, FollowerConstants.rightFrontMotorName);
        sensor3 = hardwareMap.get(DcMotorEx.class, "sensor3");
        sensor4 = hardwareMap.get(DcMotorEx.class, "sensor4");

        //TODO: see if this breaks, because you don't init it in DifferentialSwervePod but you construct it with an initialized sensor
        leftFront = new DifferentialSwervePod(new Point(0,10,Point.CARTESIAN), sensor1, false);
        rightFront = new DifferentialSwervePod(new Point(0,-10,Point.CARTESIAN), sensor2, false);
        // only necessary for three or four pod drivetrains
        leftRear = new DifferentialSwervePod(new Point(-10,0,Point.CARTESIAN), sensor3, false);
        rightRear = new DifferentialSwervePod(new Point(-10, -10,Point.CARTESIAN), sensor4, false);

        //has to contain at least two pods
        pods = Arrays.asList(leftFront, rightFront);

        Point[] podPositions = new Point[pods.size()];

        for (int i = 0; i < pods.size(); i++) {
            podPositions[i] = pods.get(i).getLocation();
        }

        centre = MathFunctions.averagePoints(podPositions);
        for (DifferentialSwervePod pod : pods) {pod.setRotateVector(centre);}

    }

    @Override
    public double[] getDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() > maxPowerScaling) correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling) headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling) pathingPower.setMagnitude(maxPowerScaling);

        // the powers for the wheel vectors
        double [] motorPowers = new double[motors.size()];

        // this contains the pathing vectors, one for each pod (heading control requires at least 2)
        Vector[] truePathingVectors = new Vector[pods.size()];

        //TODO: there is a bunch of indentation here, I think you can manage this better.
        //TODO: most of this is the same as in MecanumDrive, save for the loops. If you add those to MecanumDrive you can probably transfer a bunch of this math to Drivetrain.java
        if (correctivePower.getMagnitude() == maxPowerScaling) {
            // checks for corrective power equal to max power scaling in magnitude. if equal, then set pathing power to that
            for (int i = 0; i < pods.size(); i++) {
                truePathingVectors[i] = MathFunctions.copyVector(correctivePower);
            }
        } else {
            // corrective power did not take up all the power, so add on heading power, but as a scalar applied to the rotate vector of the pod
            // this is the only vector logic that is different from MecanumDrivetrain
            Vector[] podVectorsWithHeading = new Vector[pods.size()];
            Vector[] rotateVectorCopies = new Vector[pods.size()];
            boolean headingsTooLarge = false;

            for (int i = 0; i < pods.size(); i++) {
                rotateVectorCopies[i] = pods.get(i).getRotateVector();
                rotateVectorCopies[i].rotateVector(-robotHeading);
                rotateVectorCopies[i].setMagnitude(headingPower.getMagnitude());
                podVectorsWithHeading[i] = MathFunctions.addVectors(correctivePower, rotateVectorCopies[i]);
                // if one of the vectors is now longer than allowed, all have to be scaled down
                if (podVectorsWithHeading[i].getMagnitude() > maxPowerScaling) headingsTooLarge = true;
            }

            if (headingsTooLarge) {
                //if the combined corrective and heading power is greater than 1, then scale down heading power
                double headingScalingFactor = Math.min(findNormalizingScaling(correctivePower, headingPower), findNormalizingScaling(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, -1)));
                for (int i = 0; i < pods.size(); i++) {
                    pods.get(i).setRotateMagnitude(headingPower.getMagnitude() * headingScalingFactor);
                    truePathingVectors[i] = MathFunctions.addVectors(correctivePower, pods.get(i).getRotateVector());
                }
            } else {
                // if we're here then we can add on some drive power but scaled down to 1
                Vector[] podVectorsWithPathing = new Vector[pods.size()];
                boolean pathingsTooLarge = false;

                for (int i = 0; i < pods.size(); i++) {
                    podVectorsWithPathing[i] = MathFunctions.addVectors(podVectorsWithHeading[i], pathingPower);
                    // if one of the vectors is now longer than allowed, all have to be scaled down
                    if (podVectorsWithPathing[i].getMagnitude() > maxPowerScaling) pathingsTooLarge = true;
                }

                if (pathingsTooLarge) {
                    // too much power now, so we scale down the pathing vector
                    double pathingScalingFactor = Math.min(findNormalizingScaling(podVectorsWithHeading[0], pathingPower), findNormalizingScaling(podVectorsWithHeading[1], pathingPower));
                    //this only runs for more than two pods
                    for (int i = 2; i < pods.size(); i++) {
                        pathingScalingFactor = Math.min(pathingScalingFactor, findNormalizingScaling(podVectorsWithHeading[i], pathingPower));
                    }
                    //sets the true pathing vectors
                    for (int i = 0; i < pods.size(); i++) {
                        truePathingVectors[i] = MathFunctions.addVectors(podVectorsWithHeading[i], MathFunctions.scalarMultiplyVector(pathingPower, pathingScalingFactor));
                    }
                } else {
                    // just add the vectors together and you get the final vector
                    for (int i = 0; i < pods.size(); i++) {
                        truePathingVectors[i] = MathFunctions.copyVector(podVectorsWithPathing[i]);
                    }
                }
            }
        }

        // this bit is different then MecanumDrivetrain, as you have to calculate powers for the pods
        for (int i = 0; i < pods.size(); i++) {
            double[] podPowers = pods.get(i).calculateMotorPowers(truePathingVectors[i]);
            //TODO: maybe a less ugly way to do this
            motorPowers[2 * i] = podPowers[0];
            motorPowers[2 * i + 1] = podPowers[1];
        }

        // if the motor powers somehow get above the allowed level, scale everything down
        // I don't think this can happen in a differential swerve pod, because the drive power isn't too big (checked above), and to rotate only lowers the absolute value of one motor.
        double motorPowerMax = Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1]));
        for (int i = 2; i < pods.size(); i++) {
            motorPowerMax = Math.max(motorPowerMax, Math.abs(motorPowers[i]));
        }
        if (motorPowerMax > maxPowerScaling) {
            for (int i = 0; i < motorPowers.length; i++) {
                motorPowers[i] /= motorPowerMax;
                motors.get(i).setPower(motorPowers[i]);
            }
        }

        return motorPowers;
    }
    //TODO: remove

    public static int rotations = 0;
    double x, y, r, gamepadTheta, pidR, pidB, TICKS_PER_ROTATION = 537.7/15*26;
    int redCurrentPos, blueCurrentPos, FBpos, FRpos,BBpos,BRpos, redHeadingGoal = 0, blueHeadingGoal = 0;
    int[] encoderPositions = {FBpos,FRpos,BBpos,BRpos};
    private CustomPIDFCoefficients podPIDFCoefficients = new CustomPIDFCoefficients(
            0.0025,
            0.001,
            0.00004,
            0);

    private PIDFController blueHeading = new PIDFController(podPIDFCoefficients),
    redHeading = new PIDFController(podPIDFCoefficients);

    public void singleJoyStickPID(double x, double y) {
        Vector drive = MathFunctions.copyPointToVector(new Point(x, y, Point.CARTESIAN));
        r = drive.getMagnitude();
        gamepadTheta = drive.getTheta();

        for (int i = 0; i < motors.size(); i++) {
            encoderPositions[i] = motors.get(i).getCurrentPosition();
        }

        redCurrentPos = encoderPositions[1] - encoderPositions[3];
        blueCurrentPos = encoderPositions[0] - encoderPositions[2];


        if(r > 0.3){
            //TODO: stop rotation all the way back, maybe with gm0/FTClib button press to event?
            redHeadingGoal = (int) ((gamepadTheta-0.5*Math.PI)/(Math.PI)*TICKS_PER_ROTATION + rotations * 2 * TICKS_PER_ROTATION);
            blueHeadingGoal = (int) ((gamepadTheta-0.5*Math.PI)/(Math.PI)*TICKS_PER_ROTATION + rotations * 2 * TICKS_PER_ROTATION);
        }

        redHeading.updateError(redHeadingGoal - redCurrentPos);
        pidR = redHeading.runPIDF();

        blueHeading.updateError(blueHeadingGoal - blueCurrentPos);

        motors.get(0).setPower(pidB - r);
        motors.get(1).setPower(pidR + r);
        motors.get(2).setPower(-pidB - r);
        motors.get(3).setPower(-pidR + r);
    }
}