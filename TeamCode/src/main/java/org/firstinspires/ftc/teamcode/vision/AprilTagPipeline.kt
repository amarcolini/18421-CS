package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationIdentity
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.opencv.core.Mat
import org.openftc.easyopencv.OpenCvWebcam
import org.openftc.easyopencv.TimestampedOpenCvPipeline

class AprilTagPipeline(var calibration: () -> CameraCalibration) : TimestampedOpenCvPipeline() {
    val processor = AprilTagProcessor.Builder()
        .setDrawTagID(true)
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .build()

    override fun init(mat: Mat) {
        processor.init(mat.width(), mat.height(), calibration())
    }

    override fun processFrame(input: Mat, captureTimeNanos: Long): Mat {
        requestViewportDrawHook(
            processor.processFrame(input, captureTimeNanos)
        )
        return input
    }

    override fun onDrawFrame(
        canvas: Canvas?,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any?
    ) {
        processor.onDrawFrame(
            canvas,
            onscreenWidth,
            onscreenHeight,
            scaleBmpPxToCanvasPx,
            scaleCanvasDensity,
            userContext
        )
    }
}