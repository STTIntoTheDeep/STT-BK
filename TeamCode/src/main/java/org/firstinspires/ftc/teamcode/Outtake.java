package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController;

/**
 * TODO: documentation
 * @author Dean van Beek - 3977 Stanislas Tech Team
 */
@Config
public class Outtake {
    public enum sequenceStates {IDLING, GRAB, UP, FIDDLE, SHOULDER, SCORE, SCORED, DOWN}
    public enum slidePositions {
        DOWN(0),
        TRANSFER(220),
        HIGH_RUNG(1050),
        LOW_BASKET(1450),
        HIGH_BASKET(2300),
        CLEARS_INTAKE(500),
        CLEARS_ROBOT(750),
        LOWER_LIMIT(0),
        UPPER_LIMIT(2500);

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
    public sequenceStates specimenStates = sequenceStates.IDLING;

    double timer, slideDelta, leftPower, rightPower;

    public double leftPos, rightPos;

    final double k = 0.01;

    private int targetHeight;

    public boolean reset = false, buttonMode = false;

    public static CustomPIDFCoefficients outtakePIDFCoefficients = new CustomPIDFCoefficients(
            0.011,
            0.0,
            0.0,
            0);
    public static double ff = 0.12;

    private final PIDFController left = new PIDFController(outtakePIDFCoefficients);
    private final PIDFController right = new PIDFController(outtakePIDFCoefficients);

    public void init(HardwareMap map, boolean TeleOp) {
        hardware.motors.outtakeLeft.initMotor(map);
        hardware.motors.outtakeLeft.dcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hardware.motors.outtakeRight.initMotor(map);
        hardware.motors.outtakeRight.dcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hardware.servos.outtakeClaw.initServo(map);
        hardware.servos.shoulder.initServo(map);

        if (TeleOp) return;

        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeGrip);
        hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);

        hardware.motors.outtakeLeft.dcMotorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.outtakeLeft.dcMotorEx.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hardware.motors.outtakeRight.dcMotorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.outtakeRight.dcMotorEx.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * TODO: documentation
     * One has already transferred
     * @param high
     * @param next
     */
    public void scoreBasket(boolean high, boolean next) {
        switch (sampleStates) {
            case IDLING:
                if (next) {
                    sampleStates = sequenceStates.GRAB;
                    hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeGrip);
                    timer = System.currentTimeMillis();
                }
                break;
            case GRAB:
                if (timer + 250 < System.currentTimeMillis()) {
                    if (high) targetHeight = slidePositions.HIGH_BASKET.getPosition();
                    else targetHeight = slidePositions.LOW_BASKET.getPosition();
                    sampleStates = sequenceStates.UP;
                }
                break;
            case UP:
                slidePID(targetHeight);
                if (PIDReady() && next) sampleStates = sequenceStates.SCORE;
                break;
            case SCORE:
                slidePID(targetHeight);
                if (timer + 800 < System.currentTimeMillis()) {
                    hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
                    sampleStates = sequenceStates.DOWN;
                    targetHeight = 0;
                    timer = System.currentTimeMillis();
                }
                break;
            case DOWN:
                slidePID(targetHeight);
                if (timer + 300 < System.currentTimeMillis()) {
                    if (PIDReady()) {
                        sampleStates = sequenceStates.IDLING;
                        buttonMode = false;
                    }
                    if (next) sampleStates = sequenceStates.UP;
                }
                break;
        }
    }

    public void smashDown(boolean next) {
        switch (specimenStates) {
            case IDLING:
                if (next) {
                    specimenStates = sequenceStates.GRAB;
                    hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeGrip);
                    timer = System.currentTimeMillis();
                }
                break;
            case GRAB:
                if (timer + 250 < System.currentTimeMillis()) {
                    targetHeight = 800;
                    specimenStates = sequenceStates.UP;
                }
                break;
            case UP:
                slidePID(targetHeight);
                if (PIDReady() && next) specimenStates = sequenceStates.SHOULDER;
                break;
            case SHOULDER:
                hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderForward);
                slidePID(targetHeight);
                timer = System.currentTimeMillis();
                specimenStates = sequenceStates.SCORE;
                break;
            case SCORE:
                slidePID(targetHeight);
                if (timer + 800 < System.currentTimeMillis()) {
                    hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
                    specimenStates = sequenceStates.DOWN;
                    targetHeight = 0;
                    timer = System.currentTimeMillis();
                }
                break;
            case DOWN:
                slidePID(targetHeight);
                if (timer + 300 < System.currentTimeMillis()) {
                    hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
                    if (PIDReady()) {
                        specimenStates = sequenceStates.IDLING;
                        buttonMode = false;
                    }
                    if (next) specimenStates = sequenceStates.UP;
                }
                break;
        }
    }

    /**
     * TODO: documentation
     * @param next
     */
    public void scoreSpecimen(boolean next) {
        switch (specimenStates) {
            case IDLING:
                if (next) {
                    specimenStates = sequenceStates.GRAB;
                    hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeGrip);
                    timer = System.currentTimeMillis();
                }
                break;
            case GRAB:
                if (timer + 250 < System.currentTimeMillis()) {
                    targetHeight = 1300;
                    specimenStates = sequenceStates.UP;
                }
                break;
            case UP:
                slidePID(targetHeight);
                if (PIDReady() && next) specimenStates = sequenceStates.SHOULDER;
                break;
            case SHOULDER:
                hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderForward);
                slidePID(targetHeight);
                timer = System.currentTimeMillis();
                specimenStates = sequenceStates.SCORE;
                break;
            case SCORE:
                if (timer + 800 < System.currentTimeMillis()) {
                    slidePID(700);
                    if (hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition() < 700) {
                        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
                        specimenStates = sequenceStates.DOWN;
                        targetHeight = 0;
                        timer = System.currentTimeMillis();
                    }
                } else {
                    slidePID(targetHeight);
                }
                break;
            case DOWN:
                slidePID(targetHeight);
                if (timer + 300 < System.currentTimeMillis()) {
                    hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
                    if (PIDReady()) {
                        specimenStates = sequenceStates.IDLING;
                        buttonMode = false;
                    }
                    if (next) specimenStates = sequenceStates.UP;
                }
                break;
        }
    }

    /**
     * TODO: documentation
     * @param target
     */
    public void slidePID(double target) {
        left.setCoefficients(outtakePIDFCoefficients);
        right.setCoefficients(outtakePIDFCoefficients);
        getSlidePositions();
        left.updateError(target - leftPos);
        leftPower = left.runPIDF();
        if (leftPos > slidePositions.TRANSFER.getPosition()) {
            leftPower = Math.max(leftPower + ff, -0.7);
        } else leftPower = target < slidePositions.TRANSFER.getPosition() ? 0 : Math.max(leftPower + ff, -0.7);

        right.updateError(target - rightPos);
        rightPower = right.runPIDF();
        if (rightPos > slidePositions.TRANSFER.getPosition()) {
            rightPower = Math.max(rightPower + ff, -0.7);
        } else rightPower = target < slidePositions.TRANSFER.getPosition() ? 0 : Math.max(rightPower + ff, -0.7);
        setSlides(false);
    }

    public boolean PIDReady() {return left.getError() < 45 && right.getError() < 45;}

    /**
     * TODO: documentation
     * @param power
     */
    public void slidesWithinLimits(double power) {slidesWithinLimits(power, slidePositions.UPPER_LIMIT.getPosition());}
    /**
     * TODO: documentation
     * @param power
     */
    public void slidesWithinLimits(double power, int upperLimit) {
        getSlidePositions();
        if (power > 0) {
            if (leftPos < upperLimit) {
                leftPower = power;
                rightPower = power;
            }
            else {
                leftPower = 0;
                rightPower = 0;
            }
        } else {
            if (leftPos > slidePositions.LOWER_LIMIT.getPosition()) {
                leftPower = power;
                rightPower = power;
            }
            else {
                leftPower = 0;
                rightPower = 0;
            }
        }
        setSlides(true);
    }

    public void getSlidePositions() {
        leftPos = hardware.motors.outtakeLeft.dcMotorEx.getCurrentPosition();
        rightPos = hardware.motors.outtakeRight.dcMotorEx.getCurrentPosition();
    }

    private void setSlides(boolean compensate) {
        if (compensate) {
            slideDelta = leftPos - rightPos;
            hardware.motors.outtakeLeft.setPower(leftPower - k * slideDelta);
            hardware.motors.outtakeRight.setPower(rightPower + k * slideDelta);
        } else {
            hardware.motors.outtakeLeft.setPower(leftPower);
            hardware.motors.outtakeRight.setPower(rightPower);
        }
    }
}