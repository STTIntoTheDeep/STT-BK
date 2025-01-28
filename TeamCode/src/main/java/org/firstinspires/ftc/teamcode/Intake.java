package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;
import org.opencv.core.Scalar;

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

    private final CustomPIDFCoefficients intakePIDFCoefficients = new CustomPIDFCoefficients(
            -0.006,
            0.0,
            -0.15,
            0);

    private PIDFController pid = new PIDFController(intakePIDFCoefficients);

    public final double ticksPerCM = 45.1;

    /**
     * TODO: documentation
     * @param map
     * @param TeleOp
     */
    public void init(HardwareMap map, boolean TeleOp) {
        hardware.motors.intake.initMotor(map);
        hardware.motors.intake.dcMotorEx.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hardware.motors.intake.dcMotorEx.setDirection(DcMotorSimple.Direction.REVERSE);

        hardware.servos.intake.initServo(map);
        hardware.servos.wrist.initServo(map);
        hardware.servos.elbowLeft.initServo(map);
        hardware.servos.elbowRight.initServo(map);
        hardware.servos.keepSlide.initServo(map);

        if (TeleOp) return;
        hardware.servos.intake.setServo(hardware.servoPositions.intakeRelease.getPosition());
    }

    /**
     * TODO: documentation
     * @param color
     * @param sample
     */
    public void intake(Scalar color, SampleDetectionPipeline.Sample sample) {}

    /**
     * TODO: documentation
     * @param target
     */
    public void slidePID(double target) {
        pid.updateError(target - hardware.motors.intake.dcMotorEx.getCurrentPosition());
        hardware.motors.intake.setPower(pid.runPIDF());
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
    public void elbowToAngle(double cm) {
        hardware.servos.elbowLeft.setServo(Math.acos(cm/14.3) / (2*Math.PI) + 0.1);
        hardware.servos.elbowRight.setServo(Math.acos(cm/14.3) / (2*Math.PI) + 0.0);
    }

    /**
     * Radians, whatever the camera says is zero, is zero
     * TODO: find out
     * @param theta
     */
    public void wristToAngle(double theta) {hardware.servos.wrist.setServo(theta / (2*Math.PI) + 0.0);}

    /**
     * TODO: documentation
     * @param positions
     */
    public void setElbow(double[] positions) {
        hardware.servos.elbowLeft.setServo(positions[0]);
        hardware.servos.elbowLeft.setServo(positions[1]);
    }

    /**
     * TODO: documentation
     * @return
     */
    public boolean PIDReady() {return pid.getError() < 10;}
}