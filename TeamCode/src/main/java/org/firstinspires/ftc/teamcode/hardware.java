package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

public class hardware {
    //TODO: maybe just name things after their port and change that over here
    //TODO: test if all these this.xxx are necessary
    //TODO: documentation

    //TODO: test if this makes a noticeable difference on performance
    public static boolean reduceHardwareCalls = true;

    /**
     *
     */
    public enum motors {
        leftBack("blue_front", DcMotorSimple.Direction.REVERSE),//CH0
        leftFront("blue_back"),//CH1
        rightBack("red_front", DcMotorSimple.Direction.REVERSE),//CH3
        rightFront("red_back"),//CH2

        arm("arm", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER),//EH0
        hookLeft("hookLeft", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER),//EH1
        hookRight("hookRight", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER),//EH2
        slides("slides", DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);//EH3

        /**
         *
         * @param name
         * @param direction
         * @param mode
         */
        motors(String name, DcMotorSimple.Direction direction, DcMotor.RunMode mode) {
            this.name = name;
            this.direction = direction;
            this.mode = mode;
        }

        /**
         * default RUN_WITHOUT_ENCODER
         * @param name
         * @param direction
         */
        motors(String name, DcMotorSimple.Direction direction) {this(name, direction, DcMotor.RunMode.RUN_WITHOUT_ENCODER);}

        /**
         * default FORWARD
         * @param name
         * @param mode
         */
        motors(String name, DcMotor.RunMode mode) {this(name, DcMotorSimple.Direction.FORWARD, mode);}

        /**
         * default FORWARD, RUN_WITHOUT_ENCODER
         * @param name
         */
        motors(String name) {this(name, DcMotorSimple.Direction.FORWARD, DcMotor.RunMode.RUN_WITHOUT_ENCODER);}

        public DcMotorEx dcMotorEx;

        private final String name;

        private final DcMotorSimple.Direction direction;

        private final DcMotor.RunMode mode;

        private double lastPower = 0;

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
         *
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
    }

    /**
     *
     */
    public enum servos {
        wristLeft("wristLeft"),//CH1
        intake("intake"),//CH2
        claw("claw", 0.0),//CH4

        wristRight("wristRight"),//EH0
        keepSlide("keepSlides", 0.0);//EH1

        /**
         *
         * @param name
         * @param startPosition
         */
        servos(String name, Double startPosition) {this.name = name; this.startPosition = startPosition;}

        servos(String name){this(name, null);}

        private final String name;

        private final Double startPosition;

        private Servo servo;

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
         *
         * @return
         */
        public Double getStartPosition() {
            if (startPosition == null) return null;
            return this.startPosition;
        }

        /**
         *
         * @param map
         */
        public void initServo(HardwareMap map) {
            servo = map.get(Servo.class, getName());
            if (startPosition != null) servo.setPosition(startPosition);
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
    }
}
