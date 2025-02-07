package org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

/**
 * TODO: documentation
 * This is only a math class, it doesn't write to the motors itself.
 *
 * @author Dean van Beek - 3977 Stanislas Tech Team
 * @version 1.0, 25/12/2024 (Happy Christmas)
 */
public class DifferentialSwervePod {
    //Choose yourself if 0,0 is centre of the robot or bottom corner.
    private Point location;
    private Vector rotateVector;
    private int position;
    double theta = 0, r, rotatePower, currentAngle, targetAngle, maxPower;

    // Note: position is in ticks, and TPR converts to rotations (not radians)
    //TODO: something something absolute encoder this should be in FollowerConstants.java
    private final double TICKS_PER_ROTATION = 8192;//TODO: change
    private double encoderGearRatio = 1.0;

    private final CustomPIDFCoefficients podPIDFCoefficients = new CustomPIDFCoefficients(
            1.0,
            0.0,
            0.04,
            0);

    private PIDFController podPIDF = new PIDFController(podPIDFCoefficients);

    /** TODO documentation
     * With only two pods the distance of the point doesn't matter, only the angle.
     */
    public DifferentialSwervePod(Point location) {setLocation(location);}

    /** TODO: documentation
     *  pod with no location means location (0,0)
     */
    public DifferentialSwervePod() {setLocation(new Point(0,0,Point.CARTESIAN));}

    /**
     * TODO: documentation
     * @see <a href="#calculateMotorPowers(Vector)">calculateMotorPowers(Vector)</a> but with separate rotate
     * @param input
     * @param rotatePower
     * @return motor powers
     */
    public double[] calculateMotorPowers(Vector input, double rotatePower) {
        Vector vector = MathFunctions.addVectors(input, MathFunctions.scalarMultiplyVector(rotateVector, rotatePower));
        if (vector.getMagnitude() > 1) vector = MathFunctions.normalizeVector(vector);
        return calculateMotorPowers(vector);
    }

    /**
     * TODO: documentation
     * @param input has to be robot centric already, and the magnitude has to be clamped already
     * @return motor powers
     */
    public double[] calculateMotorPowers(Vector input) {
        if (MathFunctions.roughlyEquals(input.getMagnitude(), 0)) return new double[]{0, 0};
        double[] motorPowers = new double[2];

        theta = input.getTheta();

        currentAngle = MathFunctions.normalizeAngle(position / TICKS_PER_ROTATION * 2 * Math.PI);

        targetAngle = MathFunctions.normalizeAngle(theta - currentAngle);
        int direction = 1;
        if (targetAngle > 1.5 * Math.PI) targetAngle -= 2*Math.PI;
        else if (targetAngle > 0.5 * Math.PI) {targetAngle -= Math.PI; direction = -1;}

        podPIDF.updateError(targetAngle);
        rotatePower = MathFunctions.clamp(podPIDF.runPIDF(), -1, 1);

        r = direction * (input.getMagnitude() - rotatePower);

        motorPowers[0] = -rotatePower + r;
        motorPowers[1] = rotatePower + r;
        maxPower = Math.max(Math.abs(motorPowers[0]), Math.abs(motorPowers[1]));
        if (maxPower > 1) {
            motorPowers[0] /= maxPower;
            motorPowers[1] /= maxPower;
        }
        return motorPowers;
    }

    /**
     * TODO documentation
     * @return location Point
     */
    public Point getLocation() {return location;}

    /**
     * TODO documentation
     *
     * @param location
     */
    public void setLocation(Point location) {this.location = location;}

    /**
     * TODO documentation
     * @return Vector
     */
    public Vector getRotateVector() {return rotateVector;}

    /**
     * TODO documentation
     * turns left.
     * normalized
     *
     * @param centre
     */
    public void setRotateVector(Point centre) {
        Vector normal = new Vector(MathFunctions.subtractPoints(location,centre));
        normal.rotateVector(Math.PI*0.5);
        rotateVector = MathFunctions.copyVector(normal);
        rotateVector.setMagnitude(1);
    }

    /**
     * TODO documentation
     * @param magnitude
     */
    public void setRotateMagnitude(double magnitude) {
        rotateVector.setMagnitude(magnitude);
    }

    /**
     * TODO: documentation
     * @param position
     */
    public void setPosition(int position) {this.position = position;}
    /**
     * TODO: documentation
     * @return position in ticks
     */
    public int getPosition() {return position;}

    /**
     * TODO: documentation
     */
    public void resetPIDF() {podPIDF.reset();}

    /**
     * TODO: documentation
     * @param
     */
    public void setEncoderGearRatio(double ratio) {encoderGearRatio = ratio;}
}
