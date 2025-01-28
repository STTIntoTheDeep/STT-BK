package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PwmControl;

import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

/**
 * TODO: documentation
 * @author Dean van Beek - 3977 Stanislas Tech Team
 */
//TODO: remove @Config
@Config
public class Outtake {
    public enum sequenceStates {
        IDLING,
        UP,
        FIDDLE,
        SCORE,
        SCORED
    }
    public enum slidePositions {
        DOWN(0),
        TRANSFER(200),
        LOW_RUNG(600),
        HIGH_RUNG(1000),
        LOW_BASKET(1000),
        HIGH_BASKET(1900),
        CLEARS_ROBOT(500),
        LOWER_LIMIT(0),
        UPPER_LIMIT(2150);

        private final int position;
        public int getPosition(){return this.position;}

        slidePositions(int position) {
            this.position = position;
        }
    }

    public Outtake(HardwareMap hardwareMap, boolean TeleOp) {
        init(hardwareMap, TeleOp);
    }

    public sequenceStates sampleStates = sequenceStates.IDLING;

    private double timer, slideDelta;

    final private double k = 0.01;

    private int targetHeight;

    public boolean reset = false, buttonMode = false;

    //TODO: private final
    public static CustomPIDFCoefficients outtakePIDFCoefficients = new CustomPIDFCoefficients(
            0.4,
            0.0,
            0.015,
            0);

    private final PIDFController left = new PIDFController(outtakePIDFCoefficients);
    private final PIDFController right = new PIDFController(outtakePIDFCoefficients);

    public void init(HardwareMap map, boolean TeleOp) {
        hardware.motors.hook.initMotor(map);
        hardware.motors.hook.dcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hardware.motors.outtakeLeft.initMotor(map);
        hardware.motors.outtakeLeft.dcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hardware.motors.outtakeRight.initMotor(map);
        hardware.motors.outtakeRight.dcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hardware.motors.outtakeLeft.dcMotorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.outtakeLeft.dcMotorEx.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hardware.motors.outtakeRight.dcMotorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.outtakeRight.dcMotorEx.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hardware.servos.outtakeClaw.initServo(map);
        hardware.servos.outtakeClaw.servo.setPwmRange(new PwmControl.PwmRange(500,2500));
        hardware.servos.shoulder.initServo(map);
        hardware.servos.shoulder.servo.setPwmRange(new PwmControl.PwmRange(500,2500));

        if (TeleOp) return;

        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeGrip.getPosition());
    }

    /**
     * Auton
     * One has already transferred
     * @param high
     */
    public void scoreBasket(boolean high) {
        switch (sampleStates) {
            case IDLING:
                break;
            case UP:
                if (high) targetHeight = slidePositions.HIGH_BASKET.getPosition();
                else targetHeight = slidePositions.LOW_BASKET.getPosition();
                slidePID(targetHeight);
                if (PIDReady()) sampleStates = sequenceStates.SCORE;
                break;
            case SCORE:
                hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
                timer = System.currentTimeMillis();
                sampleStates = sequenceStates.SCORED;
                break;
            case SCORED:
                if (timer + 200 < System.currentTimeMillis()) sampleStates = sequenceStates.IDLING;
                buttonMode = false;
                break;
        }
    }

    /**
     * TeleOp
     * One has already transferred
     * @param high
     * @param score
     */
    public void scoreBasket(boolean high, boolean score) {
        switch (sampleStates) {
            case IDLING:
                break;
            case UP:
                if (high) targetHeight = slidePositions.HIGH_BASKET.getPosition();
                else targetHeight = slidePositions.LOW_BASKET.getPosition();
                slidePID(targetHeight);
                if (PIDReady() && score) sampleStates = sequenceStates.SCORE;
                break;
            case SCORE:
                hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
                timer = System.currentTimeMillis();
                sampleStates = sequenceStates.SCORED;
                break;
            case SCORED:
                if (timer + 200 < System.currentTimeMillis()) sampleStates = sequenceStates.IDLING;
                buttonMode = false;
                break;
        }
    }

    /**
     * Auton
     * @param high
     */
    public void scoreSpecimen(boolean high) {}

    /**
     * TeleOp
     * @param high
     * @param score
     */
    public void scoreSpecimen(boolean high, boolean score) {}

    /**
     * TODO: documentation
     * @param target
     */
    public void slidePID(double target) {
        left.setCoefficients(outtakePIDFCoefficients);
        right.setCoefficients(outtakePIDFCoefficients);
        left.updateError(target - hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition());
        hardware.motors.outtakeLeft.setPower(left.runPIDF());
        right.updateError(target - hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition());
        hardware.motors.outtakeRight.setPower(right.runPIDF());
    }

    public boolean PIDReady() {return left.getError() < 10 && right.getError() < 10;}

    /**
     * @param power
     * @param leftPos
     * @param rightPos
     */
    public void slidesWithinLimits(double power, double leftPos, double rightPos) {
        slideDelta = leftPos - rightPos;
        if (power > 0) {
            if (leftPos < slidePositions.UPPER_LIMIT.getPosition()) {
                hardware.motors.outtakeLeft.setPower(power - k * slideDelta);
                hardware.motors.outtakeRight.setPower(power + k * slideDelta);
            }
            else {
                hardware.motors.outtakeLeft.setPower(0);
                hardware.motors.outtakeRight.setPower(0);
            }
        } else {
            if (leftPos > slidePositions.LOWER_LIMIT.getPosition()) {
                hardware.motors.outtakeLeft.setPower(power - k * slideDelta);
                hardware.motors.outtakeRight.setPower(power + k * slideDelta);
            }
            else {
                hardware.motors.outtakeLeft.setPower(0);
                hardware.motors.outtakeRight.setPower(0);
            }
        }
    }
}