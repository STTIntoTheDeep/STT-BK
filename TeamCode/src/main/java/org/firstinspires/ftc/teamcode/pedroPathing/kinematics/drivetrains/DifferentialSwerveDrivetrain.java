package org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerConstants;

import java.util.Arrays;
import java.util.List;

/**
 * This is the DifferentialSwerveDrivetrain class, based off Pedro-Pathing's DriveVectorScaler class. This class takes in inputs Vectors for driving, heading
 * correction, and translational/centripetal correction is able to run the motors for a two-pod, three-pod or four-pod differential swerve drivetrain.
 *
 * @author Dean van Beek - 3977 Stanislas Tech Team
 * @version 1.0, 27/12/2024
 */
public class DifferentialSwerveDrivetrain extends Drivetrain {
    HardwareMap hardwareMap;

    private final List<DifferentialSwervePod> pods;
    private List<DcMotorEx> sensors;

    /**
     * This creates a new DifferentialSwerveDrivetrain, which takes in various movement vectors and outputs
     * the wheel drive powers necessary to move in the intended direction, given the true movement
     * vector for the front left mecanum wheel. It also <a href="#initialize()">initializes</a> the hardware.
     * @param hardwareMap the HardwareMap required for initialization.
     * @param pods has to contain at least two pods
     */
    public DifferentialSwerveDrivetrain(HardwareMap hardwareMap, DifferentialSwervePod ... pods) {
        this.hardwareMap = hardwareMap;
        this.pods = Arrays.asList(pods);
        separateVectors = this.pods.size();
        initialize();
    }

    // For documentation on @Override methods, see Drivetrain.java
    @Override
    public void initialize() {
        drivetrainMotors = Arrays.asList(hardware.motors.leftFront, hardware.motors.rightFront, hardware.motors.leftBack, hardware.motors.rightBack);
        for (hardware.motors motor : drivetrainMotors) {
            motor.initMotor(hardwareMap);
            motor.initPedro();
        }

        //TODO: see if this works with actual encoders
        sensors = Arrays.asList(hardware.motors.rightFront.dcMotorEx, hardware.motors.rightBack.dcMotorEx);
        for (DcMotorEx sensor : sensors) {
            sensor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sensor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        //only necessary for three or four pod drivetrains
        if (pods.size() >= 3) {
            //TODO: change if enum drivetrainMotors works
            DcMotorEx leftBackTop = hardwareMap.get(DcMotorEx.class, FollowerConstants.backLeftFrontMotorName);
            DcMotorEx leftBackBottom = hardwareMap.get(DcMotorEx.class, FollowerConstants.backLeftFrontMotorName);
            leftBackTop.setDirection(FollowerConstants.backLeftFrontMotorDirection);
            leftBackBottom.setDirection(FollowerConstants.backLeftRearMotorDirection);

            motors.add(leftBackTop);
            motors.add(leftBackBottom);
            sensors.add(leftBackBottom);
        }
        if (pods.size() >= 4) {
            DcMotorEx rightBackTop = hardwareMap.get(DcMotorEx.class, FollowerConstants.backRightFrontMotorName);
            DcMotorEx rightBackBottom = hardwareMap.get(DcMotorEx.class, FollowerConstants.backRightRearMotorName);
            rightBackTop.setDirection(FollowerConstants.backRightFrontMotorDirection);
            rightBackBottom.setDirection(FollowerConstants.backRightRearMotorDirection);

            motors.add(rightBackTop);
            motors.add(rightBackBottom);
            sensors.add(rightBackBottom);
        }

//        for (DcMotorEx motor : motors) {
//            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
//            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
//            motor.setMotorType(motorConfigurationType);
//
//            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        }

        Point[] podPositions = new Point[pods.size()];

        for (int i = 0; i < pods.size(); i++) {
            podPositions[i] = pods.get(i).getLocation();
        }

        Point centre = MathFunctions.averagePoints(podPositions);
        for (DifferentialSwervePod pod : pods) {pod.setRotateVector(centre);}
    }

    @Override
    public double[] getDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() > maxPowerScaling) correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling) headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling) pathingPower.setMagnitude(maxPowerScaling);
        robotHeading = MathFunctions.normalizeAngle(robotHeading);

        // makes the vectors robot centric
        correctivePower.rotateVector(-robotHeading);
        pathingPower.rotateVector(-robotHeading);

        // this contains the pathing vectors, one for each pod (heading control requires at least 2)
        Vector[] truePathingVectors = prioritizeVectors(correctivePower, headingPower, pathingPower, robotHeading);

        return calculateMotorPowers(truePathingVectors);
    }

    @Override
    public Vector[] applyHeadingVectors(Vector vector, Vector headingPower) {
        Vector[] rotateVectorCopies = new Vector[pods.size()];
        Vector[] vectorsWithHeading = new Vector[pods.size()];

        int turnDirection = MathFunctions.roughlyEquals(headingPower.getTheta(), robotHeading) ? 1 : -1;

        for (int i = 0; i < pods.size(); i++) {
            rotateVectorCopies[i] = MathFunctions.copyVector(pods.get(i).getRotateVector());
            rotateVectorCopies[i].setMagnitude(turnDirection * headingPower.getMagnitude());
            vectorsWithHeading[i] = MathFunctions.addVectors(vector, rotateVectorCopies[i]);
        }
        return vectorsWithHeading;
    }

    @Override
    public double[] calculateMotorPowers(Vector[] truePathingVectors) {
        // the powers for the wheels
        double[] motorPowers = new double[drivetrainMotors.size()];
        // this bit is different then MecanumDrivetrain, as you have to calculate powers for the pods
        for (int i = 0; i < pods.size(); i++) {
            pods.get(i).setPosition(sensors.get(i).getCurrentPosition());
            double[] podPowers = pods.get(i).calculateMotorPowers(truePathingVectors[i]);
            motorPowers[2 * i] = podPowers[0];
            motorPowers[2 * i + 1] = podPowers[1];
        }

        // if the motor powers somehow get above the allowed level, scale everything down
        // I don't think this can happen in a differential swerve pod, because the drive power isn't too big (checked above), and to rotate only lowers the absolute value of one motor.
        double motorPowerMax = Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1]));
        for (int i = 2; i < drivetrainMotors.size(); i++) {
            motorPowerMax = Math.max(motorPowerMax, Math.abs(motorPowers[i]));
        }
        if (motorPowerMax > maxPowerScaling) {
            for (int i = 0; i < motorPowers.length; i++) {
                motorPowers[i] /= motorPowerMax;
            }
        }
        return motorPowers;
    }
}