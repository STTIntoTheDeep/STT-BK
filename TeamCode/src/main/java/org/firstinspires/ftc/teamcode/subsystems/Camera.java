package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Path;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;

public class Camera extends Subsystem {
    public static final Camera INSTANCE = new Camera();
    private Camera() { }

    static SampleDetectionPipeline samplePipeline;

    public static double[] bestSampleInformation;

    /**
     * TODO: documentation
     * @return if it's picked a good sample or not
     */
    public static boolean chooseSample() {
        OpModeData.INSTANCE.getTelemetry().addData("count", samplePipeline.count);
        if (bestSampleInformation != null) {
            OpModeData.INSTANCE.getTelemetry().addData("x", bestSampleInformation[0]);
            OpModeData.INSTANCE.getTelemetry().addData("y", bestSampleInformation[1]);
            OpModeData.INSTANCE.getTelemetry().addData("angle", bestSampleInformation[2]);
        }
        else OpModeData.INSTANCE.getTelemetry().addLine("No best sample");
        if (samplePipeline.bestSampleInformation == null) return false;
        bestSampleInformation = samplePipeline.bestSampleInformation;

        double total = hardware.getSlideLength() + bestSampleInformation[1];

        OpModeData.INSTANCE.getTelemetry().addData("slideLength", hardware.getSlideLength());
        OpModeData.INSTANCE.getTelemetry().addData("arm Length", hardware.armLength*Math.sin(Math.acos(bestSampleInformation[0]/hardware.armLength)));
        OpModeData.INSTANCE.getTelemetry().addData("total", total);
        OpModeData.INSTANCE.getTelemetry().addData("target", total - hardware.armLength*Math.sin(Math.acos(bestSampleInformation[0]/hardware.armLength)));

        //if it's a good sample

        return checkSample();
    }

    /**
     * TODO: documentation
     * @return
     */
    protected static boolean checkSample() {
        if (Math.abs(bestSampleInformation[0]) > hardware.armLength) return false;
        if (bestSampleInformation == null) return false;
        return !hardware.tooLong(hardware.predictRobotLength(hardware.getSlideLength(), bestSampleInformation[1]));
    }

    @Override
    public void initialize() {
        samplePipeline = new SampleDetectionPipeline(false);
        hardware.initCamera(OpModeData.INSTANCE.getHardwareMap(), samplePipeline);
    }
}
