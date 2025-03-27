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
        SHOULDER,
        SCORE,
        SCORED,
        DOWN
    }
    public enum slidePositions {
        DOWN(0),
        TRANSFER(220),
        HIGH_RUNG(1050),
        LOW_BASKET(1450),
        HIGH_BASKET(2300),
        CLEARS_INTAKE(400),
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

    public sequenceStates specimenStates = sequenceStates.IDLING;

    double timer, power;

    public double leftPos, rightPos;

    private int targetHeight;

    public boolean reset = false, buttonMode = false;

    public static CustomPIDFCoefficients outtakePIDFCoefficients = new CustomPIDFCoefficients(
            0.011,
            0.0,
            0.0,
            0);
    public static double ff = 0.12;

    private final PIDFController pid = new PIDFController(outtakePIDFCoefficients);
    private final PIDFController right = new PIDFController(outtakePIDFCoefficients);

    public void init(HardwareMap map, boolean TeleOp) {
        hardware.motors.outtake.initMotor(map);
        hardware.motors.outtake.dcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        hardware.servos.outtakeClaw.initServo(map);

        if (TeleOp) return;

        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeGrip);

        hardware.motors.outtake.dcMotorEx.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        hardware.motors.outtake.dcMotorEx.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
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
                    targetHeight = 1400;
                    specimenStates = sequenceStates.UP;
                }
                break;
            case UP:
                slidePID(targetHeight);
                if (PIDReady() && next) specimenStates = sequenceStates.SHOULDER;
                break;
            case SHOULDER:
                slidePID(targetHeight);
                timer = System.currentTimeMillis();
                specimenStates = sequenceStates.SCORE;
                targetHeight = 700;
                break;
            case SCORE:
                if (timer + 200 < System.currentTimeMillis()) {
                    slidePID(targetHeight);
                    if (PIDReady()) {
                        hardware.servos.outtakeClaw.setServo(hardware.servoPositions.outtakeRelease);
                        specimenStates = sequenceStates.DOWN;
                        targetHeight = 0;
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
                break;
        }
    }

    /**
     * TODO: documentation
     * @param target
     */
    public void slidePID(double target) {
        pid.setCoefficients(outtakePIDFCoefficients);
        leftPos = hardware.motors.outtake.dcMotorEx.getCurrentPosition();
        pid.updateError(target - leftPos);
        power = pid.runPIDF();
        if (leftPos > slidePositions.TRANSFER.getPosition()) {
            power = Math.max(power + ff, -0.7);
        } else power = target < slidePositions.TRANSFER.getPosition() ? 0 : Math.max(power + ff, -0.7);

        hardware.motors.outtake.setPower(this.power);
    }

    public boolean PIDReady() {return pid.getError() < 45 && right.getError() < 45;}

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
        leftPos = hardware.motors.outtake.dcMotorEx.getCurrentPosition();
        if (power > 0) {
            if (leftPos < upperLimit) this.power = power;
            else this.power = 0;
        } else {
            if (leftPos > slidePositions.LOWER_LIMIT.getPosition()) this.power = power;
            else this.power = 0;
        }
        hardware.motors.outtake.setPower(this.power);
    }
}