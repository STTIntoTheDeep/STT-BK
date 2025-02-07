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
    private final CustomPIDFCoefficients intakePIDFCoefficients = new CustomPIDFCoefficients(
            0.02,
            0.0,
            0.0,
            0);

    private final PIDFController pid = new PIDFController(intakePIDFCoefficients);

    final double slideExtension = 72;
    public final double ticksPerCM = 756/slideExtension;
    final double baseLength = 37.0, maxLength = 108.0;
    public final double armLength = 15.1;
    double elbowAngle;

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
        pid.updateError(target - hardware.motors.intake.dcMotorEx.getCurrentPosition());
        double power = pid.runPIDF();
//        hardware.motors.intake.setPower(Math.signum(power) * Math.min(Math.abs(power), 0.7));
        hardware.motors.intake.setPower(Math.max(power, -0.5));
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
    public void elbowYDistance(double cm) {
        elbowAngle = Math.acos(cm/armLength);
        elbowAngle = hardware.servoPositions.elbowRight.getDifferential()[0] + (hardware.servoPositions.elbowLeft.getDifferential()[0] - hardware.servoPositions.elbowRight.getDifferential()[0]) * elbowAngle / Math.PI;
        setElbow(elbowAngle, hardware.servoPositions.elbowCentered.getDifferential()[1]);
    }

    /**
     * In radians, a sample which is wide from left to right (seen from the front of the robot) is angle zero.
     * This is relative to the camera, which moves with the wrist.
     * @param theta
     */
    public void wristToAngle(double theta) {
        if (theta > Math.PI) theta -= 2*Math.PI;
        hardware.servos.wrist.setServo(hardware.servoPositions.wristTransfer.getPosition() + (hardware.servoPositions.wristSpecimenCamera.getPosition() - hardware.servoPositions.wristSampleCamera.getPosition()) * theta/Math.PI);
    }

    /**
     * TODO: documentation
     * @param yaw
     * @param pitch
     */
    public void setElbow(double yaw, double pitch) {
        hardware.servos.elbowLeft.setServo(yaw - pitch);
        hardware.servos.elbowRight.setServo(yaw + pitch);
    }

    public void setElbow(double[] positions) {setElbow(positions[0], positions[1]);}

    /**
     * TODO: documentation
     * @return
     */
    public boolean PIDReady() {return pid.getError() < 5;}

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
}