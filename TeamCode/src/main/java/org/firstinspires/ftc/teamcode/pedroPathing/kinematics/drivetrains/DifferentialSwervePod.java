package org.firstinspires.ftc.teamcode.pedroPathing.kinematics.drivetrains;

import android.hardware.Sensor;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Encoder;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

/**
 * TODO: documentation
 *
 * @author Dean van Beek - 3977 Stanislas Tech Team
 * @version 1.0, 25/12/2024 (Happy Christmas)
 */
public class DifferentialSwervePod {
    //Choose yourself if 0,0 is centre of the robot or bottom corner.
    private Point location;
    private Vector rotateVector;
    private boolean absoluteSensor;
    private int rotations = 0, position;
    private double currentAngle, deltaAngle, rotatePower;

    // Note: position is in ticks, and TPR converts to rotations (not radians)
    private final double ticksPerRotation = 4096;//TODO: change
    private final double encoderGearRatio = 1.0;

    private CustomPIDFCoefficients podPIDFCoefficients = new CustomPIDFCoefficients(
            0.0025,
            0.001,
            0.00004,
            0);

    private PIDFController podPIDF = new PIDFController(podPIDFCoefficients);

    DcMotorEx sensor;

    /** TODO documentation
     */
    public DifferentialSwervePod(Point location, DcMotorEx sensor, boolean absoluteSensor) {
        setLocation(location);
        this.sensor = sensor;
        this.absoluteSensor = absoluteSensor;
    }

    /** TODO: documentation
     *  pod with no location means location zero zero
     */
    public DifferentialSwervePod() {setLocation(new Point(0,0,Point.CARTESIAN));}

    /**
     * TODO: documentation
     * @see <a href="#calculateMotorPowers(Vector)">calculateMotorPowers(Vector)</a> but with separate rotate
     * @param drive
     * @param rotatePower
     * @return motor powers
     */
    public double[] calculateMotorPowers(Vector drive, double rotatePower) {
        return calculateMotorPowers(MathFunctions.addVectors(drive, MathFunctions.scalarMultiplyVector(rotateVector, rotatePower)));
    }

    /**
     * TODO: documentation
     * @param drive has to be robot centric already, and the magnitude has to be clamped already
     * @return motor powers
     */
    public double[] calculateMotorPowers(Vector drive) {
        if (MathFunctions.roughlyEquals(drive.getMagnitude(), 0)) return new double[] {0,0};

        // update sensor data
        //FIXME
        position += sensor.getCurrentPosition();

        // calculate where the pod currently is
        //TODO: figure something out if gear ratio isn't 1:1
        if (absoluteSensor) currentAngle = position / ticksPerRotation * 2*Math.PI;
        else {
            currentAngle = (position / ticksPerRotation - rotations) * 2*Math.PI;
            //this shouldn't trigger most of the time, and it really shouldn't trigger twice, ever, but it might
            while (currentAngle > 2*Math.PI) {
                rotations--;
                currentAngle = (position / ticksPerRotation - rotations) * 2*Math.PI;
            }
            while (currentAngle < 0) {
                rotations++;
                currentAngle = (position / ticksPerRotation - rotations) * 2*Math.PI;
            }
        }

        //calculate where to go to, and if to reverse the motors (because that's faster than turning the pod half a rotation)
        deltaAngle = drive.getTheta() - currentAngle;
        if (deltaAngle > 2*Math.PI) deltaAngle -= 2*Math.PI;
        else if (deltaAngle < 0) deltaAngle += 2*Math.PI;

        rotatePower = MathFunctions.clamp(podPIDF.runPIDF(), -1, 1);

        if (deltaAngle > 0.5*Math.PI || deltaAngle < -0.5*Math.PI) return new double[] {-drive.getMagnitude(), -drive.getMagnitude() + 2 * rotatePower};
        else return new double[] {drive.getMagnitude() - 2 * rotatePower, drive.getMagnitude()};
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
     * @return position in ticks
     */
    public int getPosition() {return position;}

    /**
     * TODO: documentation
     */
    public void restPIDF() {podPIDF.reset();}
}
