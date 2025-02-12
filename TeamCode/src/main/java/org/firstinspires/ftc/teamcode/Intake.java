package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

/**
 * TODO: documentation
 * @author Dean van Beek - 3977 Stanislas Tech Team
 */
@Config
public class Intake {

    /**
     * TODO: documentation
     * @param hardwareMap
     * @param TeleOp
     */
    public Intake(HardwareMap hardwareMap, boolean TeleOp) {
        init(hardwareMap, TeleOp);
    }

    //TODO: squid?
    public static CustomPIDFCoefficients intakePIDFCoefficients = new CustomPIDFCoefficients(
            0.02,
            0.0,
            0.0,
            0);

    private final PIDFController pid = new PIDFController(intakePIDFCoefficients);

    private final double baseLength = 37.0;
    final double maxLength = 108.0, slideExtension = 72;
    public final double ticksPerCM = 756/slideExtension, armLength = 13.6;
    double elbowAngle, lastLeftPos, lastRightPos, lastWristPos;

    /**
     * TODO: documentation
     * @param map
     * @param TeleOp
     */
    public void init(HardwareMap map, boolean TeleOp) {
        hardware.motors.intake.initMotor(map);
        hardware.motors.intake.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hardware.servos.intake.initServo(map);
        hardware.servos.wrist.initServo(map);
        hardware.servos.elbowLeft.initServo(map);
        hardware.servos.elbowRight.initServo(map);

        if (TeleOp) return;
        hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease);

        hardware.motors.intake.dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.intake.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * TODO: documentation
     * TODO: voltage compensation?
     * @param target
     */
    public void slidePID(double target) {
//        hardware.motors.intake.setPower(Math.signum(power) * Math.min(Math.abs(power), 0.7));
        hardware.motors.intake.setPower(Math.max(calculatePID(target), -0.5));
    }

    public double calculatePID(double target) {
        pid.updateError(target - hardware.motors.intake.dcMotorEx.getCurrentPosition());
        return pid.runPIDF();
    }

    /**
     * TODO: documentation
     * @param cm
     */
    public void slideCM(double cm) {slidePID(cm * ticksPerCM);}

    /**
     * Centimeters perpendicular to the slides, negative is to the left
     * TODO: documentation
     * @param cm
     */
    public double elbowYDistance(double cm) {
        elbowAngle = Math.acos(cm/armLength);
        return setElbow(hardware.servoPositions.elbowRight.getDifferential()[0] + (hardware.servoPositions.elbowLeft.getDifferential()[0] - hardware.servoPositions.elbowRight.getDifferential()[0]) * elbowAngle / Math.PI,
                hardware.servoPositions.elbowRight.getDifferential()[1] + (hardware.servoPositions.elbowLeft.getDifferential()[1] - hardware.servoPositions.elbowRight.getDifferential()[1]) * elbowAngle / Math.PI);
    }

    /**
     * In radians, a sample which is wide from left to right (seen from the front of the robot) is angle zero.
     * This is relative to the camera, which moves with the wrist.
     * @param theta
     */
    public double wristToAngle(double theta) {
        double halfRotationInPos = hardware.servoPositions.wristSpecimenCamera.getPosition() - hardware.servoPositions.wristSampleCamera.getPosition();
        lastWristPos = hardware.servos.wrist.getLastPosition();

        theta = (theta * halfRotationInPos) / Math.PI;
        theta = theta + 0.5*halfRotationInPos + hardware.servoPositions.wristSampleCamera.getPosition();
        while (theta < 0.0) theta += halfRotationInPos;
        if (theta < 1.0 - halfRotationInPos) {
            if (Math.abs(theta - lastWristPos) > Math.abs(theta + halfRotationInPos - lastWristPos)) {
                theta += halfRotationInPos;
            }
        }

        while (theta > 1.0) theta -= halfRotationInPos;
        if (theta > 0.0 + halfRotationInPos) {
            if (Math.abs(theta - lastWristPos) > Math.abs(theta - halfRotationInPos - lastWristPos)) {
                theta -= halfRotationInPos;
            }
        }
        hardware.servos.wrist.setServo(theta);
        return 1200 * Math.abs(theta - lastWristPos);
    }

    /**
     * TODO: documentation
     * @param yaw
     * @param pitch
     */
    public double setElbow(double yaw, double pitch) {
        lastLeftPos = hardware.servos.elbowLeft.getLastPosition();
        lastRightPos = hardware.servos.elbowLeft.getLastPosition();
        hardware.servos.elbowLeft.setServo(yaw - pitch);
        hardware.servos.elbowRight.setServo(yaw + pitch);
        return 500*Math.max(Math.abs(hardware.servos.elbowLeft.getLastPosition() - lastLeftPos), Math.abs(hardware.servos.elbowLeft.getLastPosition() - lastRightPos));
    }

    public double setElbow(double[] positions) {return setElbow(positions[0], positions[1]);}

    /**
     * TODO: documentation
     * @return
     */
    public boolean PIDReady() {return pid.getError() < 10;}

    /**
     * TODO: documentation
     * You can only call this on special occasions
     * @return
     */
    public double getServoLength() {
        return Math.sin(elbowAngle);
    }

    /**
     * TODO: documentation
     * @return
     */
    public double getRobotLength() {return baseLength + getSlideLength() + getServoLength();}

    /**
     * TODO: documentation
     * @return
     */
    public double getSlideLength() {return hardware.motors.intake.dcMotorEx.getCurrentPosition()/ticksPerCM;}

    /**
     * TODO: documentation
     * @param slideLength
     * @param servoLength
     * @return
     */
    public double predictRobotLength(double slideLength, double servoLength) {return baseLength + slideLength + Math.sin(elbowAngle);}

    /**
     * TODO: documentation
     * @return
     */
    public boolean tooLong() {return getRobotLength() > maxLength;}

    /**
     * TODO: documentation
     * @param length
     * @return
     */
    public boolean tooLong(double length) {return length > maxLength;}

    /**
     * TODO: documentation
     * TODO: you can go further armLength isn't maximal but trigonometry
     * @param power
     */
    public void slideWithinLimits(double power) {
        if (power < 0) {
            if (baseLength + getSlideLength() + armLength < maxLength) hardware.motors.intake.setPower(power);
            else hardware.motors.intake.setPower(0);
        } else hardware.motors.intake.setPower(0);
    }
}