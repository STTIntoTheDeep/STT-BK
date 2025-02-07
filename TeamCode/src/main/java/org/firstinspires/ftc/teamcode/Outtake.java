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
    public enum sequenceStates {
        IDLING,
        GRAB,
        UP,
        FIDDLE,
        SCORE,
        SCORED,
        DOWN
    }
    public enum slidePositions {
        DOWN(0),
        TRANSFER(200),
        HIGH_RUNG(1050),
        LOW_BASKET(1450),
        HIGH_BASKET(2300),
        CLEARS_ROBOT(500),
        LOWER_LIMIT(0),
        UPPER_LIMIT(2300);

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
            0.008,
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
                    targetHeight = slidePositions.HIGH_RUNG.getPosition();
                    specimenStates = sequenceStates.UP;
                }
                break;
            case UP:
                slidePID(targetHeight);
                if (PIDReady() && next) specimenStates = sequenceStates.SCORE;
                break;
            case SCORE:
                hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderForward);
                slidePID(targetHeight);
                timer = System.currentTimeMillis();
                specimenStates = sequenceStates.SCORED;
                targetHeight -= 200;
                break;
            case SCORED:
                if (timer + 200 < System.currentTimeMillis()) {
                    slidePID(targetHeight);
                    if (next) {
                        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
                        specimenStates = sequenceStates.DOWN;
                        targetHeight = 0;
                        hardware.servos.shoulder.setServo(hardware.servoPositions.shoulderBack);
                    }
                }
                break;
            case DOWN:
                slidePID(targetHeight);
                if (PIDReady()) {
                    specimenStates = sequenceStates.IDLING;
                    buttonMode = false;
                }
                if (next) specimenStates = sequenceStates.UP;
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

    public boolean PIDReady() {return left.getError() < 50 && right.getError() < 15;}

    /**
     * @param power
     */
    public void slidesWithinLimits(double power) {
        getSlidePositions();
        if (power > 0) {
            if (leftPos < slidePositions.UPPER_LIMIT.getPosition()) {
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