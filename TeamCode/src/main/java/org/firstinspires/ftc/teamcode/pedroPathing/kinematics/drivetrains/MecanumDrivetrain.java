package org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains;

import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorDirection;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorDirection;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.leftRearMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorDirection;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightFrontMotorName;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorDirection;
import static org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants.rightRearMotorName;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.pedroPathing.kinematics.Drivetrain;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;

import java.util.Arrays;

/**
 * This is the MecanumDrivetrain class, based off Pedro-Pathings DriveVectorScaler class. This class takes in inputs Vectors for driving, heading
 * correction, and translational/centripetal correction and returns an array with wheel powers for a four wheel (four motor) mecanum drivetrain.
 * This also includes a bunch of deprecated STT TeleOp drive methods, which should be replaced with Pedro Pathings TeleOp Enhancements.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @author Aaron Yang - 10158 Scott's Bots
 * @author Harrison Womack - 10158 Scott's Bots
 * @author Dean van Beek - 3977 Stanislas Tech Team
 * @version 1.0, 24/12/2024
 */
public class MecanumDrivetrain extends Drivetrain {
    HardwareMap hardwareMap;

    private DcMotorEx leftFront,leftRear,rightFront,rightRear;

    // This is ordered left front, left back, right front, right back. These are also normalized.
    private Vector[] mecanumVectors;

    int j;

    final double angle = FollowerConstants.frontLeftVector.getTheta();

    final double[]
            baseLVector = {FollowerConstants.frontLeftVector.getXComponent(), FollowerConstants.frontLeftVector.getYComponent()},
            baseRVector = {-FollowerConstants.frontLeftVector.getXComponent(), FollowerConstants.frontLeftVector.getYComponent()},
            motorWeights = {1.0,1.0,1.0,1.0};

    public double yL,yR,minYValue,powerMultiplier,maxPower;
    public double[]
            motorPowers = {0,0,0,0},
            LVector,RVector,sumVector = {0,0};

    /**
     * This creates a new DriveVectorScaler, which takes in various movement vectors and outputs
     * the wheel drive powers necessary to move in the intended direction, given the true movement
     * vector for the front left mecanum wheel.
     */
    public MecanumDrivetrain(HardwareMap hardwareMap) {
        Vector copiedFrontLeftVector = MathFunctions.normalizeVector(FollowerConstants.frontLeftVector);
        mecanumVectors = new Vector[]{
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2*Math.PI-copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), 2*Math.PI-copiedFrontLeftVector.getTheta()),
                new Vector(copiedFrontLeftVector.getMagnitude(), copiedFrontLeftVector.getTheta())};

        this.hardwareMap = hardwareMap;
//        initialize();
    }

    @Override
    public void initialize() {
        leftFront = hardwareMap.get(DcMotorEx.class, leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, leftRearMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, rightRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, rightFrontMotorName);
        leftFront.setDirection(leftFrontMotorDirection);
        leftRear.setDirection(leftRearMotorDirection);
        rightFront.setDirection(rightFrontMotorDirection);
        rightRear.setDirection(rightRearMotorDirection);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);

            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
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

    @Deprecated
    public void init(HardwareMap map) {
        leftFront = map.get(DcMotorEx.class, "left_front");
        leftRear = map.get(DcMotorEx.class, "right_front");
        rightFront = map.get(DcMotorEx.class, "left_back");
        rightRear = map.get(DcMotorEx.class, "right_back");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Deprecated
    public void outdatedRobotCentric(double[] driveVector, double rotate) {
        motorPowers[0] = driveVector[0]*(Math.sin(driveVector[1])+Math.cos(driveVector[1])) + rotate;
        motorPowers[1] = driveVector[0]*(Math.sin(driveVector[1])-Math.cos(driveVector[1])) - rotate;
        motorPowers[2] = driveVector[0]*(Math.sin(driveVector[1])+Math.cos(driveVector[1])) - rotate;
        motorPowers[3] = driveVector[0]*(Math.sin(driveVector[1])-Math.cos(driveVector[1])) + rotate;

        setMotors();
    }

    @Deprecated
    public void robotCentric(double forward, double right, double rotate) {
        double rotateMultiplier = 0.5;
        double leftFrontPower = forward + right + rotateMultiplier*rotate;
        double rightFrontPower = forward - right - rotateMultiplier*rotate;
        double rightRearPower = forward + right - rotateMultiplier*rotate;
        double leftRearPower = forward - right + rotateMultiplier*rotate;
        double maxPower = 1.0;

        maxPower = Math.max(maxPower, Math.abs(leftFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightFrontPower));
        maxPower = Math.max(maxPower, Math.abs(rightRearPower));
        maxPower = Math.max(maxPower, Math.abs(leftRearPower));

        leftFront.setPower(leftFrontPower / maxPower);
        leftRear.setPower(rightFrontPower / maxPower);
        rightRear.setPower(rightRearPower / maxPower);
        rightFront.setPower(leftRearPower / maxPower);
    }
    /**
     * This is more useful for use in OpModes.
     * @param values An array containing a drivePower, driveAngle and rotatePower.
     * @see <a href="#drive(double[], double)">drive()</a>, values follows the same limitations.
     */
    @Deprecated
    public void robotCentric(double[] values) {robotCentric(new double[] {values[0], values[1]}, values[2]);}

    /**
     * <p>This method does the kinematics to transfer a drive vector and a rotate power into usable motor powers for a mecanum drivetrain.
     * It also corrects for the friction in mecanum wheels which causes forward to be quicker than strafe.
     * Within the method, there is documentation for what each line does.
     * If you want a visual explanation, check out Wolfpack Machina's <a href="https://www.youtube.com/watch?v=ri06orPFaKo&t=135s">"Wolfpack Movement Breakdown</a> on Youtube,
     * from 2:15 to 4:34. We do not do that last thing with the scalar, as our kinematics do not get separate perpendicular powers, but only use one drivePower.
     * It can receive these parameters from the pathFollower, the pidFollower, or a driver in Tele-Op.</p><p></p>
     * <h2>Tuning</h2>
     * <h3>Step 1</h3>
     * <p>Make sure initialization works correctly, and that all motors have been added to the configuration file.
     * If strafing doesn't drive straight, you should change the motorWeights. Downscale the side which is going too fast.</p><p></p>
     * <h3>Step 2</h3>
     * <p>Drive forwards at full speed, and drive backwards at full speed. Your angle value is the ratio forward / strafe, which is the inverse tangent.</p>
     * @param drivePower has to be a polar vector with two values: r and theta. R is a number between -1 and 1, but preferably between 0 and 1.
     *                   Theta is the angle, in radians, between the vector and the positive x-axis. 0 is forwards, 0.5 PI is to the left, PI is backwards, 1.5 PI is to the right.
     * @param rotatePower should be an already calculated power, preferably using a PID. Positive rotatePower means a counterclockwise rotation. TODO: idk if that true.
     * @see <a href="#unoptimizedDrive(double[], double)">unoptimisedDrive</a>, as it may be slightly more readable.
     */
    @Deprecated
    public void robotCentric(double[] drivePower, double rotatePower) {
        // Normalizes drivePower theta, because the math only works between -90 and 270 degrees (in radians).
        while (drivePower[1] > 1.5 * Math.PI) {
            drivePower[1] -= 2 * Math.PI;
        }
        while (drivePower[1] <= -0.5 * Math.PI) {
            drivePower[1] += 2 * Math.PI;
        }

        // Creates two new polar vectors. You only need two because the front left and back right vector are equal, and vice versa.
        // The drivePower theta is taken away from this, so that the drivePower theta aligns with the x-axis.
        // This makes getting the component of the motor vectors perpendicular to the drivePower really easy, as it's just the sine.
        LVector = new double[]{0.5 * drivePower[0], angle - drivePower[1]};
        RVector = new double[]{0.5 * drivePower[0], Math.PI - angle - drivePower[1]};

        // If the motor theta is more than 90 degrees away from the drivePower theta, it should be negated, because it's taking away from the robot speed.
        // Negating it makes the vector add to the robot speed.
        // Also, if you don't do this, you can't strafe.
        if (Math.abs(LVector[1]) > 0.5 * Math.PI) {
            LVector[0] *= -1;
        }
        if (Math.abs(RVector[1]) > 0.5 * Math.PI) {
            RVector[0] *= -1;
        }

        // This gets the smallest y-component from both motor vectors. The reason for this might seem vague, but the vector with the larger y-component is scaled down,
        // so the forces perpendicular to the drivePower cancel each other out, and you drive in the direction of the drivePower vector.
        yL = Math.abs(LVector[0] * Math.sin(LVector[1]));
        yR = Math.abs(RVector[0] * Math.sin(RVector[1]));
        minYValue = Math.min(yL, yR);
        LVector[0] *= minYValue / yL;
        RVector[0] *= minYValue / yR;

        // This code adds the vectors together. To do this easily, you need cartesian coordinates.
        sumVector = MathFunctions.cartesianToPolar(LVector[0]*Math.cos(LVector[1])+RVector[0]*Math.cos(RVector[1]),LVector[0]*Math.sin(LVector[1])+RVector[0]*Math.sin(RVector[1]));
        //Alternative sumVector from https://www.youtube.com/watch?v=vr71A_UFt0A. No clue if this is more efficient or not. Definitely not as readable.
//        sumVector[1] = RVector[1]-LVector[1]+Math.PI;
//        sumVector[0] = Math.sqrt(LVector[0]*LVector[0]*RVector[0]*RVector[0]-2*LVector[0]*RVector[0]*Math.cos(sumVector[1]));
//        sumVector[1] = Math.asin(RVector[0]*Math.sin(sumVector[1])/sumVector[0]);

        // SumVector theta should be exactly the drivePower theta, but r might differ.
        // We then multiply by that difference, so that the sumVector is completely equal to the driveVector.
        //TODO: can you live without this correction
        // I have a feeling you can delete or optimise this bit, because we correct for the same thing again later and the drive vectors aren't changed from the beginning.
        // If you can delete this, do, because extra calculation sucks.
        powerMultiplier = drivePower[0]/sumVector[0];
        LVector[0] = LVector[0] * powerMultiplier;
        RVector[0] = RVector[0] * powerMultiplier;
        sumVector[0] = sumVector[0] * powerMultiplier;

        // Next, the motor powers are calculated. As stated before, the vector for front left and back right is the same, but the rotation is different.
        // You also might want to add a different weight for each motor, this might be useful when your robots center of gravity is very much to one side.
        motorPowers[0] = (LVector[0] - rotatePower) * motorWeights[0];
        motorPowers[1] = (RVector[0] + rotatePower) * motorWeights[1];
        motorPowers[2] = (RVector[0] - rotatePower) * motorWeights[2];
        motorPowers[3] = (LVector[0] + rotatePower) * motorWeights[3];

        // All motors are scaled in such a way that the largest power is the same value as the length of the drivePower parameter.
        maxPower = Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1]));
        maxPower = Math.max(maxPower, Math.abs(motorPowers[2]));
        maxPower = Math.max(maxPower, Math.abs(motorPowers[3]));
        for (int i = 0; i < 4; i++) {
            motorPowers[i] /= (maxPower*drivePower[0]);
        }
        setMotors();
    }

    /**
     * @see <a href="#robotCentric(double[], double)">robotCentric(double[], double)</a>. This method does the same thing in a slightly more comprehensible, but slower way.
     */
    @Deprecated
    public void unoptimizedDrive(double[] drivePower, double rotatePower) {
        LVector = new double[]{baseLVector[0],baseLVector[1]-drivePower[1]};
        RVector = new double[]{baseRVector[0],baseRVector[1]-drivePower[1]};

        if (Math.abs(LVector[1]) > 0.5 * Math.PI) {
            LVector[0] *= -1;
        }
        if (Math.abs(RVector[1]) > 0.5 * Math.PI) {
            RVector[0] *= -1;
        }

        yL = Math.abs(MathFunctions.polarToCartesian(LVector)[1]);
        yR = Math.abs(MathFunctions.cartesianToPolar(RVector)[1]);
        minYValue = Math.min(yL, yR);
        LVector[0] *= minYValue/ yL;
        RVector[0] *= minYValue/ yR;

        sumVector = MathFunctions.cartesianToPolar(2 * MathFunctions.cartesianToPolar(LVector)[0] + 2 * MathFunctions.cartesianToPolar(RVector)[0],
                2 * MathFunctions.cartesianToPolar(LVector)[1] + 2 * MathFunctions.cartesianToPolar(RVector)[1]);
        powerMultiplier = 2*drivePower[0]/sumVector[0];
        LVector[0] = LVector[0] * powerMultiplier;
        RVector[0] = RVector[0] * powerMultiplier;
        sumVector[0] = sumVector[0] * powerMultiplier;

        motorPowers[0] = (LVector[0] - rotatePower) * motorWeights[0];
        motorPowers[1] = (RVector[0] + rotatePower) * motorWeights[1];
        motorPowers[2] = (RVector[0] - rotatePower) * motorWeights[2];
        motorPowers[3] = (LVector[0] + rotatePower) * motorWeights[3];

        maxPower = Math.max(Math.abs(motorPowers[0]),Math.abs(motorPowers[1]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[2]));
        maxPower = Math.max(maxPower,Math.abs(motorPowers[3]));

        for (int i = 0; i < 4;i++) {
            motorPowers[i] /= (maxPower*drivePower[0]);
        }

        leftFront.setPower(motorPowers[0]);
        leftRear.setPower(motorPowers[1]);
        rightFront.setPower(motorPowers[2]);
        rightRear.setPower(motorPowers[3]);
    }
    private void setMotors() {
        // Motor powers are sent to the motors.
        j = 0;
        for (DcMotorEx motor : new DcMotorEx[]{leftFront, leftRear, rightFront, rightRear}) {
            motor.setPower(motorPowers[j]);
            j++;
        }
    }
}
