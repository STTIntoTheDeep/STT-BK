package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

/**
 * This is the hardware class. You can use this as an interface for all your hardware reads and writes, so you can more easily manage and minimise them.
 * I hope that because of the static-ness of enumerators, this reduces NullPointExceptions.
 * If you save the motors and servo's in the order of their hardware ports, it dramatically saves time when your config gets wiped.
 * Also, because leftBack.getName is already visibly the name of leftBack, you don't need to have the name be "left_back", it can just be motor0 or something based on the port.
 * @author Dean van Beek - 3977 Stanislas Tech Team
 * @version 0.5, 28/12/2024 (untested)
 */
public class hardware {
    //TODO: test if all these this.xxx are necessary

    public static boolean reduceHardwareCalls = true;

    /**
     * This enumerator contains all the motors, preferably in the order of their hardware ports.
     * They have a config name, direction and mode, which can all easily be initialized with initMotor.
     */
    public enum motors {
        leftBack("left_back", DcMotorSimple.Direction.REVERSE),//For swerve forward
        leftFront("left_front", DcMotorSimple.Direction.REVERSE),
        rightBack("right_back"),
        rightFront("right_front"),//For swerve forward

        hook("hook", DcMotorSimple.Direction.REVERSE, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER),
        intake("intake", DcMotorSimple.Direction.REVERSE, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER),
        outtakeLeft("outtakeLeft", DcMotorSimple.Direction.REVERSE, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER),
        outtakeRight("outtakeRight", DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        /**
         * This creates a new motor, with a config name, a direction and a RunMode.
         * @param name the name you use in the config on the Driver Hub.
         * @param direction the direction you want the motor to spin in.
         * @param mode the RunMode in which the motor needs to run.
         */
        motors(String name, DcMotorSimple.Direction direction, DcMotorEx.RunMode mode) {
            this.name = name;
            this.direction = direction;
            this.mode = mode;
        }

        /**
         * This creates a new motor, with a config name, a direction and a RunMode defaulted as RUN_WITHOUT_ENCODER.
         * @see <a href="#motors(String, DcMotorSimple.Direction, DcMotor.Runmode)">the standard constructor.</a>
         */
        motors(String name, DcMotorSimple.Direction direction) {this(name, direction, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);}

        /**
         * This creates a new motor, with a config name, a direction defaulted as FORWARD and a RunMode.
         * @see <a href="#motors(String, DcMotorSimple.Direction, DcMotor.Runmode)">the standard constructor.</a>

         */
        motors(String name, DcMotor.RunMode mode) {this(name, DcMotorSimple.Direction.FORWARD, mode);}

        /**
         * This creates a new motor, with a config name, a direction defaulted as FORWARD and a RunMode defaulted as RUN_WITHOUT_ENCODER.
         * @see <a href="#motors(String, DcMotorSimple.Direction, DcMotor.Runmode)">the standard constructor.</a>
         */
        motors(String name) {this(name, DcMotorSimple.Direction.FORWARD, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);}

        // these are the variables each motors() has.
        //TODO: make function getCurrentPos and private the motor.
        public DcMotorEx dcMotorEx;

        private final String name;

        private final DcMotorSimple.Direction direction;

        private final DcMotorEx.RunMode mode;

        private double lastPower = 0;

        //TODO: documentation for everything below this.
        /**
         *
         * @return
         */
        public String getName() {return this.name;}

        /**
         *
         * @return
         */
        public DcMotorSimple.Direction getDirection() {return this.direction;}

        /**
         *
         * @return
         */
        public DcMotor.RunMode getMode() {return this.mode;}

        /**
         *
         * @return
         */
        public double getLastPower() {return this.lastPower;}

        /**
         * TODO: instead of adding HardwareMap here, which gets added from the OpMode to the Drivetrain to here, add it from OpMode once (because static class)
         * @param map
         */
        public void initMotor(HardwareMap map) {
            dcMotorEx = map.get(DcMotorEx.class, getName());
            dcMotorEx.setDirection(getDirection());
            dcMotorEx.setMode(getMode());
        }

        /**
         *
         * @param power
         */
        public void setPower(double power) {
            if (MathFunctions.roughlyEquals(power, getLastPower()) && reduceHardwareCalls) return;
            this.dcMotorEx.setPower(power);
            this.lastPower = power;
        }

        /**
         *
         */
        public void initPedro() {
            MotorConfigurationType motorConfigurationType = this.dcMotorEx.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            this.dcMotorEx.setMotorType(motorConfigurationType);

            //TODO: ask driver if he prefers .BRAKE in TeleOp
            this.dcMotorEx.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        }
    }

    /**
     *
     */
    public enum servos {
        elbowLeft("elbowLeft"),
        intake("intakeClaw", 600, 2400),
        outtakeClaw("claw", 600, 2400),
        wrist("wrist"),
        shoulder("shoulder"),
        elbowRight("elbowRight");

        /**
         *
         * @param name
         * @param usLower
         * @param usUpper
         */
        servos(String name, int usLower, int usUpper) {this.name = name; PwmRange = new PwmControl.PwmRange(usLower,usUpper);}

        servos(String name){this(name, 500, 2500);}

        private final String name;

        private final PwmControl.PwmRange PwmRange;

        private ServoImplEx servo;

        private double lastPosition = 0;

        /**
         *
         * @return
         */
        public String getName() {return this.name;}

        /**
         *
         * @return
         */
        public double getLastPosition() {return this.lastPosition;}

        /**
         * TODO: documentation, link GM0
         * @param usLower
         * @param usUpper
         */
        public void setPwmRange(int usLower, int usUpper) {servo.setPwmRange(new PwmControl.PwmRange(usLower, usUpper));}

        /**
         *
         * @param map
         */
        public void initServo(HardwareMap map) {
            servo = map.get(ServoImplEx.class, getName());
            servo.setPwmRange(PwmRange);
        }

        /**
         * to reduce hardware calls
         * @param position
         */
        public void setServo(double position) {
            if (MathFunctions.roughlyEquals(position, getLastPosition()) && reduceHardwareCalls) return;
            servo.setPosition(position);
            this.lastPosition = position;
        }

        /**
         * Is a shorthand, but not usable for the differential or other combined servo positions.
         */
        public void setServo(servoPositions position){setServo(position.getPosition());}

    }

    /**
     *
     */
    public enum servoPositions {
        intakeGrip(0.76),
        intakeRelease(0.44),
        wristTransfer(0.39),
        wristSampleCamera(0.119),
        wristSpecimenCamera(0.71),
        keepSlides(0.5),
        releaseSlides(0),

        outtakeGrip(0.54),
        outtakeRelease(0.0),
        shoulderTransfer(0.02),
        shoulderForward(0.82),
        shoulderBack(0.34),

        elbowLeft(new double[] {0.77,-0.06}),
        elbowRight(new double[] {0.265,-0.04}),
        elbowCentered(new double[]{0.51,-0.04}),
        elbowTransfer(new double[]{0.51, 0.41}),
        cameraDown(new double[]{0.51,0.18}),
        cameraWide(new double[]{0.51,0.34});

        servoPositions(double position) {this.position = position;}
        private double position;
        public double getPosition(){return this.position;}

        servoPositions(double[] differential){this.differential = differential;}
        private double[] differential;
        public double[] getDifferential(){return this.differential;}
    }
}
