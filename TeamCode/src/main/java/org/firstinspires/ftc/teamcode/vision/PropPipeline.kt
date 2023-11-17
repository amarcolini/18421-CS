package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import com.acmerobotics.dashboard.config.variable.ConfigVariable
import com.acmerobotics.dashboard.config.variable.CustomVariable
import com.amarcolini.joos.dashboard.ConfigUtils
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.dashboard.MutableConfigProvider
import org.opencv.core.Core
import org.opencv.core.Mat
import org.opencv.core.Rect
import org.opencv.core.Scalar
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

    private fun Rect.mirror() = Rect(640 - x - width, y, width, height)

    val midRect = Rect(100, 200, 250, 200).run {
        if (leftIsVisible) this.mirror() else this
    }
    val sideRect = Rect(520, 200, 640 - 520, 480 - 200).run {
        if (leftIsVisible) this.mirror() else this
    }
    var lowerBound = Scalar(0.0, 100.0, 50.0)
    var upperBound = Scalar(255.0, 255.0, 255.0)

    companion object {
        var showProcess = true

        @MutableConfigProvider(1)
        @JvmStatic
        fun scalarConfigProvider(scalar: Scalar): ConfigVariable<*> = CustomVariable().apply {
            putVariable(
                "v0",
                ConfigUtils.createVariable({ scalar.`val`[0] }, { scalar.`val`[0] = it })
            )
            putVariable(
                "v1",
                ConfigUtils.createVariable({ scalar.`val`[1] }, { scalar.`val`[1] = it })
            )
            putVariable(
                "v2",
                ConfigUtils.createVariable({ scalar.`val`[2] }, { scalar.`val`[2] = it })
            )
        }
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
        assert(mat.size().width == 640.0 && mat.size().height == 480.0)
    }

    private data class DrawData(
        val midMean: Double,
        val sideMean: Double
    )

    override fun processFrame(frame: Mat): Mat {
        Imgproc.cvtColor(frame, dst, Imgproc.COLOR_BGR2HSV)
        Core.inRange(dst, lowerBound, upperBound, tmp)
        Core.extractChannel(dst, dst, 1)
        Core.multiply(dst, tmp, dst)
        val midMean = Core.sumElems(dst.submat(midRect)).`val`[0] / 1e5
        val sideMean = Core.sumElems(dst.submat(sideRect)).`val`[0] / 1e5
        val location: PropLocation = when {
            midMean <= 15 && sideMean <= 15 -> if (leftIsVisible) PropLocation.Right else PropLocation.Left
            sideMean > midMean -> if (leftIsVisible) PropLocation.Left else PropLocation.Right
            else -> PropLocation.Center
        }
        lastKnownLocation = location
        requestViewportDrawHook(DrawData(midMean, sideMean))
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