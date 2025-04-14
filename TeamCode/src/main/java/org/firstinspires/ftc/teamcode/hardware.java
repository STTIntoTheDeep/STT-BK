package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

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
        leftFront("left_front"),
        rightBack("right_back", DcMotorSimple.Direction.REVERSE),
        rightFront("right_front", DcMotorSimple.Direction.REVERSE),

        hook("hook", DcMotorSimple.Direction.REVERSE, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER),
        intake("intake", DcMotorSimple.Direction.REVERSE, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER),
        outtake("outtake", DcMotorSimple.Direction.REVERSE, DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

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

        public ServoImplEx servo;

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
    public enum touchSensors {
        leftFrontBumper("leftFrontSensor"),
        rightFrontBumper("rightFrontSensor"),
        armDown("armDown"),
        armUp("armUp"),
        leftBackBumper("leftBackSensor"),
        rightBackBumper("rightBackSensor"),
        clawGrab("clawGrab"),
        clawScore("clawScore");

        /**
         *
         * @param name
         */
        touchSensors(String name) {this.name = name;}

        private final String name;

        private TouchSensor sensor;

        /**
         *
         * @return
         */
        public String getName() {return this.name;}

        /**
         *
         * @param map
         */
        public void initSensor(HardwareMap map) {
            sensor = map.get(TouchSensor.class, name);
        }

        public boolean pressed() {return sensor.isPressed();}
    }

    public static void initSensors(HardwareMap map) {
        for (touchSensors sensor : touchSensors.values()) {
            sensor.initSensor(map);
        }
    }

    //TODO: repair sensor
    public static boolean touchingSubmersible() {return touchSensors.leftFrontBumper.pressed() || touchSensors.rightFrontBumper.pressed();}
    public static boolean touchingBorder() {return touchSensors.leftBackBumper.pressed() && touchSensors.rightBackBumper.pressed();}


    /**
     *
     */
    public enum servoPositions {
        intakeGrip(.91),
        intakeRelease(.48),
        wristTransfer(.615),
        wristSampleCamera(.37),
        wristSpecimenCamera(.87),
        keepSlides(.5),
        releaseSlides(.0),

        outtakeGrip(.31),
        outtakeRelease(.7),
        outtakeClear(.9),

        elbowLeft(new double[] {0.76,-0.09}),
        elbowRight(new double[] {0.26,-0.063}),
        elbowCentered(new double[]{0.52,-0.07}),
        elbowTransfer(new double[]{0.52, 0.41}),
        elbowDrop(new double[]{0.2,0.185}),
        cameraDown(new double[]{0.52,0.185}),
        cameraWide(new double[]{0.52,0.28});

        servoPositions(double position) {this.position = position;}
        private double position;
        public double getPosition(){return this.position;}

        servoPositions(double[] differential){this.differential = differential;}
        private double[] differential;
        public double[] getDifferential(){return this.differential;}
    }

    public static OpenCvCamera camera;

    public static void initCamera(HardwareMap map, OpenCvPipeline startPipeline) {
        int cameraMonitorViewId = map.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", map.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(map.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(CAMERA_WIDTH,CAMERA_HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
                camera.setPipeline(startPipeline);
            }

            @Override
            public void onError(int errorCode) {}
        });
    }

    private static final int CAMERA_WIDTH = 640; // width  of wanted camera resolution
    private static final int CAMERA_HEIGHT = 360; // height of wanted camera resolution

    public static final double
            xPixels = 640,
            yPixels = 360,
            yDegreePerPixel = 16 * Math.sqrt(3025.0/337.0) / yPixels,//14:25 ratio on the camera * sqrt ( 55 degrees squared / (14^2 + 25^2) ) = horizontal FOV, divided by pixels to get degree per pixel TODO maybe regression better
            xDegreePerPixel = 9 * Math.sqrt(3025.0/337.0) / xPixels; //TODO maybe regression better
    public static double
            cameraXPos = 11.6 , //In init, primary axis (x is forwards/backwards) offset versus the differential shaft of the intake
            cameraYPos = -0.7, //Offset in secondary axis versus the differential shaft of the intake
            cameraZPos = 28.9, //Height of the camera, relative to the floor
            cameraAlpha = 0.0; //In degrees, will be converted to radians later, 0 means parallel to the floor

    public static final double baseLength = 37.0;
    public static final double maxLength = 42*2.54, slideExtension = 72.0;
    public static final double ticksPerCM = 756/slideExtension,
            armLength = 13.7,
            fullLength = armLength + 5.0,
            armHeight = 10.3,
            clawLength = 3.7,
            halfYawRotationInPos = servoPositions.elbowLeft.getDifferential()[0] - servoPositions.elbowRight.getDifferential()[0],
            quarterPitchRotationInPos = servoPositions.cameraDown.getDifferential()[1] - servoPositions.elbowCentered.getDifferential()[1];


    /**
     * TODO: documentation
     * @return
     */
    public static double predictArmLength(double pitch, double yaw) {
        //Turn pos into angles, 0 pitch is down (longest), 0 yaw is forward (longest)
        pitch = (pitch - servoPositions.elbowCentered.getDifferential()[1]) * Math.PI * 0.5 / quarterPitchRotationInPos;
        yaw = (yaw - servoPositions.elbowCentered.getDifferential()[0]) * Math.PI / halfYawRotationInPos;
        double height = (MathFunctions.roughlyEquals(servos.intake.getLastPosition(), servoPositions.intakeGrip.getPosition())) ? armHeight : armHeight - clawLength;

        return Math.cos(yaw) * fullLength * Math.cos(pitch) + Math.sin(pitch) * height;
    }

    public static double getArmLength() {
        double yaw = (servos.elbowLeft.getLastPosition() + servos.elbowRight.getLastPosition())/2.0;
        double pitch = servos.elbowRight.getLastPosition() - servos.elbowLeft.getLastPosition();
        return predictArmLength(pitch, yaw);
    }

    /**
     * TODO: documentation
     * @return
     */
    public static double getRobotLength() {return predictRobotLength(getSlideLength(), getArmLength());}

    /**
     * TODO: documentation
     * @param ticks
     * @return
     */
    public static double predictSlideLength(int ticks) {return ticks/hardware.ticksPerCM;}

    /**
     * TODO: documentation
     * @return
     */
    public static double getSlideLength() {return predictSlideLength(hardware.motors.intake.dcMotorEx.getCurrentPosition());}

    /**
     * TODO: documentation
     * @param slideLength
     * @param servoLength
     * @return
     */
    public static double predictRobotLength(double slideLength, double servoLength) {
        return hardware.baseLength + slideLength + Math.max(servoLength, 0);
    }

    /**
     * TODO: documentation
     * @return
     */
    public static boolean tooLong() {return getRobotLength() > hardware.maxLength;}

    /**
     * TODO: documentation
     * @param length
     * @return
     */
    public static boolean tooLong(double length) {return length > hardware.maxLength;}
}
