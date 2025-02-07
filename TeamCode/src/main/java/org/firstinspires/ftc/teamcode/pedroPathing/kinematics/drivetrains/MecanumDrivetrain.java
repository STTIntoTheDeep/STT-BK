package org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.FollowerConstants;

import java.util.Arrays;

/**
 * This is the MecanumDrivetrain class, based off Pedro-Pathings DriveVectorScaler class. This class takes in inputs Vectors for driving, heading
 * correction, and translational/centripetal correction and returns an array with wheel powers for a four wheel (four motor) mecanum drivetrain.
 * For documentation on @Override methods, see Drivetrain.java
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Dean van Beek - 3977 Stanislas Tech Team
 * @version 1.0, 24/12/2024
 */
public class MecanumDrivetrain extends Drivetrain {
    HardwareMap hardwareMap;

    // This is ordered left front, left back, right front, right back. These are also normalized.
    private final Vector[] mecanumVectors;

    /**
     * This creates a new DriveVectorScaler, which takes in various movement vectors and outputs
     * the wheel drive powers necessary to move in the intended direction, given the true movement
     * vector for the front left mecanum wheel. It also <a href="#initialize()">initializes</a> the hardware.
     * @param hardwareMap the HardwareMap required for initialization.
     */
    public MecanumDrivetrain(HardwareMap hardwareMap) {
        Vector copiedFrontLeftVector = MathFunctions.normalizeVector(FollowerConstants.frontLeftVector);
        mecanumVectors = new Vector[]{
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2*Math.PI-copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2*Math.PI-copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())};

        // Mecanum only requires two separate vectors for 4 wheels, because this is only necessary for the rotate vectors,
        // and that is just left backwards, right forwards. leftFrontPower and leftBackPower are different because of the mecanum
        // wheel orientation, but that is covered by mecanumVectors at the end.
        separateVectors = 2;

        this.hardwareMap = hardwareMap;
        initialize();
    }

    @Override
    public void initialize() {
        drivetrainMotors = Arrays.asList(hardware.motors.leftFront, hardware.motors.leftBack, hardware.motors.rightFront, hardware.motors.rightBack);
        for (hardware.motors motor : drivetrainMotors) {
            motor.initMotor(hardwareMap);
            motor.initPedro();
        }
    }

    @Override
    public double[] getDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() > maxPowerScaling) correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling) headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling) pathingPower.setMagnitude(maxPowerScaling);

        // the powers for the wheel vectors
        double [] wheelPowers = new double[4];

        // This contains a copy of the mecanum wheel vectors
        Vector[] mecanumVectorsCopy = new Vector[4];

        // this contains the pathing vectors, one for each side (heading control requires 2)
        Vector[] truePathingVectors = new Vector[2];

        if (correctivePower.getMagnitude() == maxPowerScaling) {
            // checks for corrective power equal to max power scaling in magnitude. if equal, then set pathing power to that
            truePathingVectors[0] = MathFunctions.copyVector(correctivePower);
            truePathingVectors[1] = MathFunctions.copyVector(correctivePower);
        } else {
            // corrective power did not take up all the power, so add on heading power
            Vector leftSideVector = MathFunctions.subtractVectors(correctivePower, headingPower);
            Vector rightSideVector = MathFunctions.addVectors(correctivePower, headingPower);

            if (leftSideVector.getMagnitude() > maxPowerScaling || rightSideVector.getMagnitude() > maxPowerScaling) {
                //if the combined corrective and heading power is greater than 1, then scale down heading power
                double headingScalingFactor = Math.min(findNormalizingScaling(correctivePower, headingPower), findNormalizingScaling(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, -1)));
                truePathingVectors[0] = MathFunctions.subtractVectors(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, headingScalingFactor));
                truePathingVectors[1] = MathFunctions.addVectors(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, headingScalingFactor));
            } else {
                // if we're here then we can add on some drive power but scaled down to 1
                Vector leftSideVectorWithPathing = MathFunctions.addVectors(leftSideVector, pathingPower);
                Vector rightSideVectorWithPathing = MathFunctions.addVectors(rightSideVector, pathingPower);

                if (leftSideVectorWithPathing.getMagnitude() > maxPowerScaling || rightSideVectorWithPathing.getMagnitude() > maxPowerScaling) {
                    // too much power now, so we scale down the pathing vector
                    double pathingScalingFactor = Math.min(findNormalizingScaling(leftSideVector, pathingPower), findNormalizingScaling(rightSideVector, pathingPower));
                    truePathingVectors[0] = MathFunctions.addVectors(leftSideVector, MathFunctions.scalarMultiplyVector(pathingPower, pathingScalingFactor));
                    truePathingVectors[1] = MathFunctions.addVectors(rightSideVector, MathFunctions.scalarMultiplyVector(pathingPower, pathingScalingFactor));
                } else {
                    // just add the vectors together and you get the final vector
                    truePathingVectors[0] = MathFunctions.copyVector(leftSideVectorWithPathing);
                    truePathingVectors[1] = MathFunctions.copyVector(rightSideVectorWithPathing);
                }
            }
        }
        // [STT] no idea why this is. Maybe because of the multiplication later.
        truePathingVectors[0] = MathFunctions.scalarMultiplyVector(truePathingVectors[0], 2.0);
        truePathingVectors[1] = MathFunctions.scalarMultiplyVector(truePathingVectors[1], 2.0);

        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            // this copies the vectors from mecanumVectors but creates new references for them
            mecanumVectorsCopy[i] = MathFunctions.copyVector(mecanumVectors[i]);

            mecanumVectorsCopy[i].rotateVector(robotHeading);
        }

        wheelPowers[0] = (mecanumVectorsCopy[1].getXComponent()*truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent()*mecanumVectorsCopy[1].getYComponent()) / (mecanumVectorsCopy[1].getXComponent()*mecanumVectorsCopy[0].getYComponent() - mecanumVectorsCopy[0].getXComponent()*mecanumVectorsCopy[1].getYComponent());
        wheelPowers[1] = (mecanumVectorsCopy[0].getXComponent()*truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent()*mecanumVectorsCopy[0].getYComponent()) / (mecanumVectorsCopy[0].getXComponent()*mecanumVectorsCopy[1].getYComponent() - mecanumVectorsCopy[1].getXComponent()*mecanumVectorsCopy[0].getYComponent());
        wheelPowers[2] = (mecanumVectorsCopy[3].getXComponent()*truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent()*mecanumVectorsCopy[3].getYComponent()) / (mecanumVectorsCopy[3].getXComponent()*mecanumVectorsCopy[2].getYComponent() - mecanumVectorsCopy[2].getXComponent()*mecanumVectorsCopy[3].getYComponent());
        wheelPowers[3] = (mecanumVectorsCopy[2].getXComponent()*truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent()*mecanumVectorsCopy[2].getYComponent()) / (mecanumVectorsCopy[2].getXComponent()*mecanumVectorsCopy[3].getYComponent() - mecanumVectorsCopy[3].getXComponent()*mecanumVectorsCopy[2].getYComponent());

        double wheelPowerMax = Math.max(Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])), Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3])));
        if (wheelPowerMax > maxPowerScaling) {
            wheelPowers[0] /= wheelPowerMax;
            wheelPowers[1] /= wheelPowerMax;
            wheelPowers[2] /= wheelPowerMax;
            wheelPowers[3] /= wheelPowerMax;
        }

        return wheelPowers;
    }

    @Override
    public Vector[] applyHeadingVectors(Vector vector, Vector headingPower) {
        Vector[] vectorsWithHeading = new Vector[2];
        vectorsWithHeading[0] = MathFunctions.subtractVectors(vector, headingPower);
        vectorsWithHeading[1] = MathFunctions.addVectors(vector, headingPower);
        return vectorsWithHeading;
    }
    @Override
    public double[] calculateMotorPowers(Vector[] truePathingVectors) {
        truePathingVectors[0] = MathFunctions.scalarMultiplyVector(truePathingVectors[0], 2.0);
        truePathingVectors[1] = MathFunctions.scalarMultiplyVector(truePathingVectors[1], 2.0);

        double [] wheelPowers = new double[4];

        Vector[] mecanumVectorsCopy = new Vector[4];

        for (int i = 0; i < mecanumVectorsCopy.length; i++) {
            // this copies the vectors from mecanumVectors but creates new references for them
            mecanumVectorsCopy[i] = MathFunctions.copyVector(mecanumVectors[i]);

            mecanumVectorsCopy[i].rotateVector(robotHeading);
        }

        // [STT] I don't understand this.
        wheelPowers[0] = (mecanumVectorsCopy[1].getXComponent()*truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent()*mecanumVectorsCopy[1].getYComponent()) / (mecanumVectorsCopy[1].getXComponent()*mecanumVectorsCopy[0].getYComponent() - mecanumVectorsCopy[0].getXComponent()*mecanumVectorsCopy[1].getYComponent());
        wheelPowers[1] = (mecanumVectorsCopy[0].getXComponent()*truePathingVectors[0].getYComponent() - truePathingVectors[0].getXComponent()*mecanumVectorsCopy[0].getYComponent()) / (mecanumVectorsCopy[0].getXComponent()*mecanumVectorsCopy[1].getYComponent() - mecanumVectorsCopy[1].getXComponent()*mecanumVectorsCopy[0].getYComponent());
        wheelPowers[2] = (mecanumVectorsCopy[3].getXComponent()*truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent()*mecanumVectorsCopy[3].getYComponent()) / (mecanumVectorsCopy[3].getXComponent()*mecanumVectorsCopy[2].getYComponent() - mecanumVectorsCopy[2].getXComponent()*mecanumVectorsCopy[3].getYComponent());
        wheelPowers[3] = (mecanumVectorsCopy[2].getXComponent()*truePathingVectors[1].getYComponent() - truePathingVectors[1].getXComponent()*mecanumVectorsCopy[2].getYComponent()) / (mecanumVectorsCopy[2].getXComponent()*mecanumVectorsCopy[3].getYComponent() - mecanumVectorsCopy[3].getXComponent()*mecanumVectorsCopy[2].getYComponent());

        double wheelPowerMax = Math.max(Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])), Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3])));
        if (wheelPowerMax > maxPowerScaling) {
            wheelPowers[0] /= wheelPowerMax;
            wheelPowers[1] /= wheelPowerMax;
            wheelPowers[2] /= wheelPowerMax;
            wheelPowers[3] /= wheelPowerMax;
        }

        return wheelPowers;
    }
}