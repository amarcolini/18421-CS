package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import com.amarcolini.joos.dashboard.JoosConfig
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

@JoosConfig
class PropPipeline : OpenCvPipeline() {
    private var tmp: Mat = Mat.zeros(Size(640.0, 480.0), 0)

    enum class PropLocation {
        Left, Center, Right
    }

    var lastKnownLocation: PropLocation = PropLocation.Center
        private set

    companion object {
        val midRect = Rect(0, 200, 500, 200)
        val rightRect = Rect(520, 200, 640 - 520, 480 - 200)
        var thresh = 100.0
    }

    private fun strokeRect(
        thickness: Float,
        color: Int,
        rect: Rect,
        canvas: Canvas,
        scaleBmpPxToCanvasPx: Float
    ) {
        val coords = listOf(rect.x, rect.y, rect.br().x, rect.br().y).map {
            (it.toDouble() * scaleBmpPxToCanvasPx).toInt()
        }.toIntArray()
        canvas.drawRect(
            android.graphics.Rect(
                coords[0], coords[1], coords[2], coords[3]
            ), Paint().apply {
                this.color = color
                style = Paint.Style.STROKE
                strokeWidth = thickness
            }
        )
    }

    override fun init(mat: Mat) {
        assert(mat.size().width == 640.0 && mat.size().height == 480.0)
    }

    private data class DrawData(
        val midMean: Double,
        val rightMean: Double
    )

    override fun processFrame(frame: Mat): Mat {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2HSV)
        Core.extractChannel(frame, frame, 1)
        Imgproc.threshold(frame, tmp, thresh, 1.0, Imgproc.THRESH_BINARY)
        Core.multiply(frame, tmp, frame)
        val midMean = Core.sumElems(frame.submat(midRect)).`val`[0] / 1e5
        val rightMean = Core.sumElems(frame.submat(rightRect)).`val`[0] / 1e5
        val location: PropLocation = when {
            midMean <= 10 && rightMean <= 10 -> PropLocation.Left
            rightMean > midMean -> PropLocation.Right
            else -> PropLocation.Center
        }
        lastKnownLocation = location
        requestViewportDrawHook(DrawData(midMean, rightMean))
        return frame
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any
    ) {
        strokeRect(2f, Color.RED, midRect, canvas, scaleBmpPxToCanvasPx)
        strokeRect(2f, Color.BLUE, rightRect, canvas, scaleBmpPxToCanvasPx)
        (userContext as? DrawData)?.let {
            canvas.drawText(
                "midMean: ${it.midMean}, rightMean: ${it.rightMean}",
                0f,
                0f,
                Paint().apply {
                    color = Color.GREEN
                    strokeWidth = 3f
                })
        }
    }
}