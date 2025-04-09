package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.groups.ParallelRaceGroup;
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController;
import com.rowanmcalpin.nextftc.core.control.controllers.feedforward.StaticFeedforward;
import com.rowanmcalpin.nextftc.ftc.OpModeData;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.HoldPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition;
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.SetPower;

import org.firstinspires.ftc.teamcode.hardware;

public class Arm extends Subsystem {
    public static final Arm INSTANCE = new Arm();
    // USER CODE
    public MotorEx motor;
    boolean armDown, previousArmDown;
    double timer = 0;

    public PIDFController controller = new PIDFController(0.006, 0.0, 0.0, new StaticFeedforward(0), 20);
//new ArmFeedforward(0.3, ticksToAngle -> 750/Math.PI)

    public static Command reset() {
        return new InstantCommand(() -> {
            hardware.motors.outtake.dcMotorEx.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hardware.motors.outtake.dcMotorEx.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        });
    }

    public Command toDown() {
        return new ParallelRaceGroup(
                new WaitUntil(hardware.touchSensors.armDown::pressed).then(reset()),
                toPosition(0)
        );
    }

    public Command toHigh() {
        return toPosition(430);
    }

    public Command toPosition(double position) {
        return new RunToPosition(motor,
                position,
                controller,
                this);
    }

    public Command holdPosition() {return new HoldPosition(motor, controller, this);}

    public Command setPower(double power) {
        return new SetPower(motor, power, this);
    }

    @Override
    public void initialize() {
        hardware.motors.outtake.initMotor(OpModeData.INSTANCE.getHardwareMap());
        motor = new MotorEx(hardware.motors.outtake.dcMotorEx);
    }

    @Override
    public void periodic() {
        if (timer + 250 < System.currentTimeMillis()) {
            timer += 250;
            previousArmDown = armDown;
            armDown = hardware.touchSensors.armDown.pressed();
            if (armDown && !previousArmDown) reset();
        }
        OpModeData.telemetry.addData("pos", motor.getCurrentPosition());
//        OpModeData.telemetry.update();
    }
}
