package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import com.amarcolini.joos.dashboard.JoosConfig
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline
import kotlin.math.roundToInt

@JoosConfig
class PropPipeline(private val leftIsVisible: Boolean) : OpenCvPipeline() {
    enum class PropLocation {
        Left, Center, Right
    }

    var lastKnownLocation: PropLocation = PropLocation.Center
        private set

    private lateinit var dst: Mat
    private lateinit var tmp: Mat

    private fun Rect.mirror() = Rect(imageSize.width.toInt() - x - width, y, width, height)

    private val midRect = mRect.run {
        if (!leftIsVisible) this.mirror() else this
    }
    private val sideRect = sRect.run {
        if (!leftIsVisible) this.mirror() else this
    }

    companion object {
        val imageSize = Size(800.0, 600.0)
        val mRect = Rect(350, 300, 250, 200)
        val sRect = Rect(0, 280, 220, 250)
        private val lowerBound = Scalar(0.0, 100.0, 50.0)
        private val upperBound = Scalar(255.0, 255.0, 255.0)
        var showProcess = true
    }

    private fun strokeRect(
        thickness: Double,
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
                strokeWidth = (thickness * scaleBmpPxToCanvasPx).toFloat()
            }
        )
    }

    override fun init(mat: Mat) {
        dst = Mat.zeros(mat.size(), mat.type())
        tmp = Mat.zeros(mat.size(), mat.type())
        assert(mat.size().width == imageSize.width && mat.size().height == imageSize.height)
    }

    private data class DrawData(
        val midMean: Double,
        val sideMean: Double
    )

    override fun processFrame(frame: Mat): Mat {
        val tempMats = ArrayList<Mat>()
        Imgproc.cvtColor(frame, dst, Imgproc.COLOR_BGR2HSV)
        Core.inRange(dst, lowerBound, upperBound, tmp)
        Core.extractChannel(dst, dst, 1)
        Core.multiply(dst, tmp, dst)
        val midMean = Core.sumElems(dst.submat(midRect).also { tempMats += it }).`val`[0] / 1e5
        val sideMean = Core.sumElems(dst.submat(sideRect).also { tempMats += it }).`val`[0] / 1e5
        val location: PropLocation = when {
            midMean <= 20 && sideMean <= 20 -> if (leftIsVisible) PropLocation.Right else PropLocation.Left
            sideMean > midMean -> if (leftIsVisible) PropLocation.Left else PropLocation.Right
            else -> PropLocation.Center
        }
        lastKnownLocation = location
        requestViewportDrawHook(DrawData(midMean, sideMean))
        tempMats.forEach { it.release() }
        return if (showProcess) dst else frame
    }

    override fun onDrawFrame(
        canvas: Canvas,
        onscreenWidth: Int,
        onscreenHeight: Int,
        scaleBmpPxToCanvasPx: Float,
        scaleCanvasDensity: Float,
        userContext: Any
    ) {
        strokeRect(2.0, Color.GREEN, midRect, canvas, scaleBmpPxToCanvasPx)
        strokeRect(2.0, Color.BLUE, sideRect, canvas, scaleBmpPxToCanvasPx)
        (userContext as? DrawData)?.let {
            canvas.drawText(
                "midMean: ${it.midMean.roundToInt()}, sideMean: ${it.sideMean.roundToInt()}",
                0f,
                25 * scaleBmpPxToCanvasPx,
                Paint().apply {
                    color = Color.RED
                    strokeWidth = 3 * scaleBmpPxToCanvasPx
                    textSize = 25 * scaleBmpPxToCanvasPx
                })
        }
    }
}