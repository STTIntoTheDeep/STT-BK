package org.firstinspires.ftc.teamcode.subsystems;

import com.rowanmcalpin.nextftc.core.Subsystem;
import com.rowanmcalpin.nextftc.ftc.OpModeData;

import org.firstinspires.ftc.teamcode.hardware;
import org.firstinspires.ftc.teamcode.vision.SampleDetectionPipeline;

public class Camera extends Subsystem {
    public static final Camera INSTANCE = new Camera();
    private Camera() { }

    public static SampleDetectionPipeline samplePipeline = new SampleDetectionPipeline(false);

    public static double[] bestSampleInformation;

    /**
     * TODO: documentation
     * Look for sample
     * @return if it's picked a good sample or not
     */
    public static boolean chooseSample() {
        OpModeData.telemetry.addData("count", samplePipeline.count);
        if (bestSampleInformation != null) {
            OpModeData.telemetry.addData("x", bestSampleInformation[0]);
            OpModeData.telemetry.addData("y", bestSampleInformation[1]);
            OpModeData.telemetry.addData("angle", bestSampleInformation[2]);
        }
        else OpModeData.telemetry.addLine("No best sample");
        if (samplePipeline.bestSampleInformation == null) return false;
        bestSampleInformation = samplePipeline.bestSampleInformation;

        double total = hardware.getSlideLength() + bestSampleInformation[1];

        OpModeData.telemetry.addData("slideLength", hardware.getSlideLength());
        OpModeData.telemetry.addData("arm Length", hardware.armLength*Math.sin(Math.acos(bestSampleInformation[0]/hardware.armLength)));
        OpModeData.telemetry.addData("total", total);
        OpModeData.telemetry.addData("target", total - hardware.armLength*Math.sin(Math.acos(bestSampleInformation[0]/hardware.armLength)));
        OpModeData.telemetry.update();

        //if it's a good sample
        if (Math.abs(bestSampleInformation[0]) > hardware.armLength) return false;
        if (bestSampleInformation == null) return false;
        return !hardware.tooLong(hardware.predictRobotLength(hardware.getSlideLength(), bestSampleInformation[1]));
    }

    @Override
    public void initialize() {
        hardware.initCamera(OpModeData.INSTANCE.getHardwareMap(), samplePipeline);
    }
}
