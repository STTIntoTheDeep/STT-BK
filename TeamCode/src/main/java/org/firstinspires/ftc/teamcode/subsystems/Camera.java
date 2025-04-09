package org.firstinspires.ftc.teamcode.subsystems;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.core.command.Command;
import com.rowanmcalpin.nextftc.core.command.utility.delays.WaitUntil;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;
import org.opencv.core.Scalar;

public class Camera extends Subsystem {
    public static final Camera INSTANCE = new Camera();
    int runs = 0;

    public SampleDetectionPipeline samplePipeline = new SampleDetectionPipeline(true);

    public double[] bestSampleInformation;

    public double slideTarget;

    /**
     * TODO: documentation
     * Look for sample
     * @return if it's picked a good sample or not
     */
    private boolean chooseSample() {
//        samplePipeline.saveRAM = false;
        bestSampleInformation = samplePipeline.externalBestSampleInformation;
        if (samplePipeline.externalBestSampleInformation == null) {
            return false;
        }
        slideTarget = hardware.getSlideLength() + samplePipeline.externalBestSampleInformation[1] - hardware.armLength*Math.sin(Math.acos(-samplePipeline.externalBestSampleInformation[0]/hardware.armLength));

        //if it's a good sample
        if (Math.abs(samplePipeline.externalBestSampleInformation[0]) > hardware.armLength) return false;
        return !hardware.tooLong(hardware.predictRobotLength(slideTarget, samplePipeline.externalBestSampleInformation[1]));
//        if (hardware.tooLong(hardware.predictRobotLength(slideTarget, samplePipeline.externalBestSampleInformation[1]))) return false;
//        samplePipeline.saveRAM = true;
//        return true;
    }

    public void cameraDownValues(){
        hardware.cameraXPos = 8.5;
        hardware.cameraYPos = -0.7;
        hardware.cameraZPos = 28.9;
        hardware.cameraAlpha = 0.0;
        SampleDetectionPipeline.AREA_LOWER_LIMIT = 15000;
        SampleDetectionPipeline.AREA_UPPER_LIMIT = 100000;
        SampleDetectionPipeline.YELLOW_MASK_THRESHOLD = 140;
        SampleDetectionPipeline.BLUE_MASK_THRESHOLD = 150;
        SampleDetectionPipeline.RED_MASK_THRESHOLD = 170;
    }
    public void cameraWideValues(){
        hardware.cameraXPos = -3;
        hardware.cameraYPos = -0.7;
        hardware.cameraZPos = 30.2;
        hardware.cameraAlpha = (hardware.servoPositions.cameraWide.getDifferential()[1] - hardware.servoPositions.cameraDown.getDifferential()[1]) * 90 / (hardware.servoPositions.elbowTransfer.getDifferential()[1] - hardware.servoPositions.cameraDown.getDifferential()[1]);
        SampleDetectionPipeline.AREA_LOWER_LIMIT = 5000;
        SampleDetectionPipeline.AREA_UPPER_LIMIT = 20000;
        SampleDetectionPipeline.YELLOW_MASK_THRESHOLD = 80;
        SampleDetectionPipeline.BLUE_MASK_THRESHOLD = 150;
        SampleDetectionPipeline.RED_MASK_THRESHOLD = 170;
    }

    public Command locateSampleSimple() {
        return new WaitUntil(this::chooseSample);
    }

    @Override
    public void initialize() {
        hardware.initCamera(OpModeData.INSTANCE.getHardwareMap(), samplePipeline);
        samplePipeline.saveRAM = true;
    }

    @Override
    public void periodic() {
        runs++;
        OpModeData.telemetry.addData("saveRAM", samplePipeline.saveRAM);
        OpModeData.telemetry.addData("camera white screening", !samplePipeline.cameraWorking);
        OpModeData.telemetry.addData("count", samplePipeline.count);
        OpModeData.telemetry.addData("telemetry runs", runs);
        OpModeData.telemetry.addData("pipeline runs", samplePipeline.runs);
        if (samplePipeline.externalBestSampleInformation == null) {OpModeData.telemetry.addLine("pipeline is null");}
        else {
            OpModeData.telemetry.addLine("pipeline isn't null");
            OpModeData.telemetry.addData("x",samplePipeline.externalBestSampleInformation[0]);
            OpModeData.telemetry.addData("y",samplePipeline.externalBestSampleInformation[1]);
            OpModeData.telemetry.addData("angle",samplePipeline.externalBestSampleInformation[2]);
        }
        if (bestSampleInformation == null) {OpModeData.telemetry.addLine("sample is null");}
        else {OpModeData.telemetry.addLine("sample isn't null");}

        if (samplePipeline.desiredColor == SampleDetectionPipeline.YELLOW) OpModeData.telemetry.addLine("yellow");
        else if (samplePipeline.desiredColor == SampleDetectionPipeline.BLUE) OpModeData.telemetry.addLine("blue");
        else if (samplePipeline.desiredColor == SampleDetectionPipeline.RED) OpModeData.telemetry.addLine("red");

        OpModeData.telemetry.addData("slideTarget", slideTarget);
        OpModeData.telemetry.addData("area", SampleDetectionPipeline.AREA_LOWER_LIMIT);

        OpModeData.telemetry.update();
    }
}
