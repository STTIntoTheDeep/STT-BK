package org.firstinspires.ftc.teamcode.pedroPathing.kinematics;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;

import java.util.List;

/**
 * This is the Drivetrain class. Based off the Localizer structure, it is an abstract superclass of all drivetrains used in STT Pedro Pathing,
 * so it contains abstract methods that will have a concrete implementation in the subclasses. Any
 * method that all drivetrains will need will be in this class.
 * Note: Contrary to the Localizer, STT only added documentation here, since that carries over into implementations of the methods. DRY.
 *
 * @author Dean van Beek
 * @version 1.0, 31/01/2025
 */
public abstract class Drivetrain {
    public List<DcMotorEx> motors;

    public List<hardware.motors> drivetrainMotors;
    private double[] motorPowers;
    protected double maxPowerScaling = 1, robotHeading;
    protected int separateVectors;

    /**
     * This method initializes the hardware necessary for the Drivetrain. It specifically initializes all the motors and adds them to a List.
     */
    protected abstract void initialize();

    /**
     * TODO: documentation
     * @param correctivePower
     * @param headingPower
     * @param pathingPower
     * @param robotHeading
     */
    public void run(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        // clamps down the magnitudes of the input vectors
        if (correctivePower.getMagnitude() > maxPowerScaling) correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling) headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling) pathingPower.setMagnitude(maxPowerScaling);
        this.robotHeading = MathFunctions.normalizeAngle(robotHeading);

        // makes the vectors robot centric
        correctivePower.rotateVector(-robotHeading);
        pathingPower.rotateVector(-robotHeading);

        // this contains the pathing vectors, one for each pod (heading control requires at least 2)
        Vector[] truePathingVectors = prioritizeVectors(correctivePower, headingPower, pathingPower, robotHeading);

//        motorPowers = getDrivePowers(correctivePower, headingPower, pathingPower, robotHeading);
        //TODO: if holding point and standing in place, don't do the feedforward thing, turn off motors completely
        motorPowers = calculateMotorPowers(truePathingVectors);
        setMotors();
    }

    /**
     * For robot centric TeleOp without centripetal force correction or testing opModes.
     * @see .FIXME
     * @param drive
     * @param rotate
     */
    public void run(Vector drive, double rotate) {run(new Vector(0,0), new Vector(rotate,0), drive, rotate);}

    /**
     * [NOTE]: This is a separate function so it can return when the vectors are done, instead of having a lot
     * of nested else-statements.
     * TODO: documentation
     * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
     *                        translational power Vector to correct onto the Bézier curve the Follower
     *                        is following.
     * @param headingPower this Vector points in the direction of the robot's current heading, and
     *                     the magnitude tells the robot how much it should turn and in which
     *                     direction.
     * @param pathingPower this Vector points in the direction the robot needs to go to continue along
     *                     the Path.
     * @param robotHeading this is the current heading of the robot, which is used to calculate how
     *                     much power to allocate to each wheel.
     * @return this returns an Array of Vectors, which contains the wheel powers.
     */
    public Vector[] prioritizeVectors(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        Vector[] truePathingVectors = new Vector[separateVectors];

        //TODO: most of this is the same as in MecanumDrive, save for the loops. If you add those to MecanumDrive you can probably transfer a bunch of this math to Drivetrain.java
        if (correctivePower.getMagnitude() == maxPowerScaling) {
            // checks for corrective power equal to max power scaling in magnitude. if equal, then set the power to that and exit the method
            for (int i = 0; i < separateVectors; i++) {
                truePathingVectors[i] = MathFunctions.copyVector(correctivePower);
            }
            return truePathingVectors;
        }

        // corrective power did not take up all the power, so add on heading power, but as a scalar applied to the rotate vector of the pod
        // this is the only vector logic that is different per drivetrain
        Vector[] podVectorsWithHeading = applyHeadingVectors(correctivePower, headingPower);

        // if one of the vectors is now longer than allowed, heading needs to be scaled down, then exit the method
        if (MathFunctions.vectorsTooBig(podVectorsWithHeading, maxPowerScaling)) {
            // too much power now, so we scale down the heading vector
            double headingScalingFactor = Math.min(findNormalizingScaling(correctivePower, headingPower), findNormalizingScaling(correctivePower, MathFunctions.scalarMultiplyVector(headingPower, -1)));
            truePathingVectors = applyHeadingVectors(correctivePower, new Vector(headingPower.getMagnitude() * headingScalingFactor, headingPower.getTheta()));
            return truePathingVectors;
        }

        // if we're here then we can add on some drive power but scaled down to 1
        Vector[] podVectorsWithPathing = new Vector[separateVectors];

        for (int i = 0; i < separateVectors; i++) {
            podVectorsWithPathing[i] = MathFunctions.addVectors(podVectorsWithHeading[i], pathingPower);
        }

        // if one of the vectors is now longer than allowed, pathing needs to be scaled down, then exit the method
        if (MathFunctions.vectorsTooBig(podVectorsWithPathing, maxPowerScaling)) {
            // too much power now, so we scale down the pathing vector
            double pathingScalingFactor = Math.min(findNormalizingScaling(podVectorsWithHeading[0], pathingPower), findNormalizingScaling(podVectorsWithHeading[1], pathingPower));
            //this only runs for more than two pods
            for (int i = 2; i < separateVectors; i++) {
                pathingScalingFactor = Math.min(pathingScalingFactor, findNormalizingScaling(podVectorsWithHeading[i], pathingPower));
            }
            //sets the true pathing vectors
            for (int i = 0; i < separateVectors; i++) {
                truePathingVectors[i] = MathFunctions.addVectors(podVectorsWithHeading[i], MathFunctions.scalarMultiplyVector(pathingPower, pathingScalingFactor));
            }
            return truePathingVectors;
        }

        // if nothing overflows, just add the vectors together and you get the final vector
        for (int i = 0; i < separateVectors; i++) {
            truePathingVectors[i] = MathFunctions.copyVector(podVectorsWithPathing[i]);
        }
        return truePathingVectors;
    }

    /**
     * TODO: documentation
     * @param vector
     * @param headingPower
     * @return
     */
    public abstract Vector[] applyHeadingVectors(Vector vector, Vector headingPower);

    /**
     * TODO: documentation
     * @param truePathingVectors
     * @return
     */
    public abstract double[] calculateMotorPowers(Vector[] truePathingVectors);


        /**
         * This takes in field centric vectors for corrective power, heading power, and pathing power and outputs
         * an Array of four doubles, one for each wheel's motor power.
         * IMPORTANT NOTE: all vector inputs are clamped between 0 and 1 inclusive in magnitude.
         *
         * @param correctivePower this Vector includes the centrifugal force scaling Vector as well as a
         *                        translational power Vector to correct onto the Bézier curve the Follower
         *                        is following.
         * @param headingPower this Vector points in the direction of the robot's current heading, and
         *                     the magnitude tells the robot how much it should turn and in which
         *                     direction.
         * @param pathingPower this Vector points in the direction the robot needs to go to continue along
         *                     the Path.
         * @param robotHeading this is the current heading of the robot, which is used to calculate how
         *                     much power to allocate to each wheel.
         * @return this returns an Array of doubles, which contains the wheel powers.
         */
    @Deprecated
    public abstract double[] getDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading);

    /**
     * This takes in two Vectors, one static and one variable, and returns the scaling factor that,
     * when multiplied to the variable Vector, results in magnitude of the sum of the static Vector
     * and the scaled variable Vector being the max power scaling. <br/>
     * IMPORTANT NOTE: I did not intend for this to be used for anything other than the method above
     * this one in this class, so there will be errors if you input Vectors of length greater than maxPowerScaling,
     * and it will scale up the variable Vector if the magnitude of the sum of the two input Vectors
     * isn't greater than maxPowerScaling. So, just don't use this elsewhere. There's gotta be a better way to do
     * whatever you're trying to do.<br/>
     * I know that this is used outside of this class, however, I created this method so I get to
     * use it if I want to. Also, it's only used once outside of the DriveVectorScaler class, and
     * it's used to scale Vectors, as intended.
     *
     * @param staticVector the Vector that is held constant.
     * @param variableVector the Vector getting scaled to make the sum of the input Vectors have a
     *                       magnitude of maxPowerScaling.
     * @return returns the scaling factor for the variable Vector.
     */
    public double findNormalizingScaling(Vector staticVector, Vector variableVector) {
            double a = Math.pow(variableVector.getXComponent(), 2) + Math.pow(variableVector.getYComponent(), 2);
            double b = staticVector.getXComponent() * variableVector.getXComponent() + staticVector.getYComponent() * variableVector.getYComponent();
            double c = Math.pow(staticVector.getXComponent(), 2) + Math.pow(staticVector.getYComponent(), 2) - Math.pow(maxPowerScaling, 2);
            return (-b + Math.sqrt(Math.pow(b, 2) - a*c))/(a);
    }

    /**
     * Sets the maximum power that can be used by the drive vector scaler. Clamped between 0 and 1.
     *
     * @param maxPowerScaling setting the max power scaling
     */
    public void setMaxPowerScaling(double maxPowerScaling) {
        this.maxPowerScaling = MathFunctions.clamp(maxPowerScaling, 0, 1);
    }

    /**
     * Gets the maximum power that can be used by the drive vector scaler. Ranges between 0 and 1.
     *
     * @return returns the max power scaling
     */
    public double getMaxPowerScaling() {
        return maxPowerScaling;
    }

    /**
     * TODO: documentation
     */
    public void setMotors() {
        for (int i = 0; i < drivetrainMotors.size(); i++) {
            drivetrainMotors.get(i).setPower(motorPowers[i]);
        }
    }

    /**
     * TODO: documentation
     */
    public void stopMotors() {
        for (int i = 0; i < drivetrainMotors.size(); i++) {
            drivetrainMotors.get(i).setPower(0);
        }
    }
}