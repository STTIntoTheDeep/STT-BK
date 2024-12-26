package org.firstinspires.ftc.teamcode.pedroPathing.kinematics;

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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

import java.util.Arrays;
import java.util.List;

/**
 * This is the Drivetrain class. Based off the Localizer structure, it is an abstract superclass of all drivetrains used in STT Pedro Pathing,
 * so it contains abstract methods that will have a concrete implementation in the subclasses. Any
 * method that all drivetrains will need will be in this class.
 * Note: Contrary to the Localizer, STT only added documentation here, since that carries over into implementations of the methods. DRY.
 *
 * @author Dean van Beek
 * @version 1.0, 24/12/2024
 */
public abstract class Drivetrain {
    protected List<DcMotorEx> motors;
    private double[] motorPowers;

    protected double maxPowerScaling = 1;

    /**
     * TODO: documentation
     */
    public abstract void initialize();

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
    public abstract double[] getDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading);

    /**
     * This takes in two Vectors, one static and one variable, and returns the scaling factor that,
     * when multiplied to the variable Vector, results in magnitude of the sum of the static Vector
     * and the scaled variable Vector being the max power scaling.
     *
     * IMPORTANT NOTE: I did not intend for this to be used for anything other than the method above
     * this one in this class, so there will be errors if you input Vectors of length greater than maxPowerScaling,
     * and it will scale up the variable Vector if the magnitude of the sum of the two input Vectors
     * isn't greater than maxPowerScaling. So, just don't use this elsewhere. There's gotta be a better way to do
     * whatever you're trying to do.
     *
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
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setPower(motorPowers[i]);
        }
    }

    /**
     * TODO: documentation
     */
    public void resetMotors() {
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).setPower(0);
        }
    }
}
