package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.VisionProcessor
import org.opencv.calib3d.Calib3d
import org.opencv.core.*
import org.opencv.imgproc.Imgproc
import kotlin.math.sqrt

class PalibrationProcessor : VisionProcessor {
    val lastFrame = Mat()
    val lastPoints = Mat()
    var isGood = false
        private set
    val patternSize = Size(10.0, 10.0)
    private val numCorners = (patternSize.width * patternSize.height).toInt()
    val objPoints = MatOfPoint3f()
    val squareSize = 1.0
    private val flags = Calib3d.CALIB_FIX_PRINCIPAL_POINT +
            Calib3d.CALIB_ZERO_TANGENT_DIST +
            Calib3d.CALIB_FIX_ASPECT_RATIO +
            Calib3d.CALIB_FIX_K4 +
            Calib3d.CALIB_FIX_K5;

    init {
        for (i in 0 until patternSize.height.toInt()) for (j in 0 until patternSize.width.toInt()) objPoints.push_back(
            MatOfPoint3f(
                Point3(
                    j * squareSize,
                    i * squareSize,
                    0.0
                )
            )
        )
    }

    private var imageSize = Size(1.0, 1.0)
    override fun init(p0: Int, p1: Int, p2: CameraCalibration?) {
        imageSize = Size(p0.toDouble(), p1.toDouble())
    }

    override fun processFrame(p0: Mat, p1: Long): Any {
        p0.copyTo(lastFrame)
        val imgPoints = MatOfPoint2f()
        isGood = Calib3d.findChessboardCorners(p0, patternSize, imgPoints)
        Calib3d.drawChessboardCorners(p0, patternSize, imgPoints, isGood)
        Imgproc.cornerSubPix(
            p0,
            lastPoints,
            Size(11.0, 11.0),
            Size(-1.0, -1.0),
            TermCriteria(TermCriteria.EPS + TermCriteria.COUNT, 30, 0.1)
        )
        return p0
    }

    private val cornersBuffer: List<Mat> = ArrayList()
    private val cameraMatrix = Mat()
    private val distortionCoefficients = Mat()
    var rms = 0.0
        private set

    fun calibrateCamera(cornerPoints: List<Mat>) {
        val rvecs = ArrayList<Mat>()
        val tvecs = ArrayList<Mat>()
        val reprojectionErrors = Mat()
        val objectPoints = ArrayList<Mat>()
        objectPoints.add(Mat.zeros(numCorners, 1, CvType.CV_32FC3))
        calcBoardCornerPositions(objectPoints[0])
        for (i in 1 until cornersBuffer.size) {
            objectPoints.add(objectPoints[0])
        }

        Calib3d.calibrateCamera(
            objectPoints, cornersBuffer, imageSize,
            cameraMatrix, distortionCoefficients, rvecs, tvecs, flags
        )

//        mIsCalibrated = (Core.checkRange(cameraMatrix)
//                && Core.checkRange(distortionCoefficients))

        rms = computeReprojectionErrors(objectPoints, rvecs, tvecs, reprojectionErrors)
    }

    private fun computeReprojectionErrors(
        objectPoints: List<Mat>,
        rvecs: List<Mat>, tvecs: List<Mat>, perViewErrors: Mat
    ): Double {
        val cornersProjected = MatOfPoint2f()
        var totalError = 0.0
        var error: Double
        val viewErrors = FloatArray(objectPoints.size)

        val distortionCoefficients = MatOfDouble(distortionCoefficients)
        var totalPoints = 0
        for (i in objectPoints.indices) {
            val points = MatOfPoint3f(objectPoints[i])
            Calib3d.projectPoints(
                points, rvecs[i], tvecs[i],
                cameraMatrix, distortionCoefficients, cornersProjected
            )
            error = Core.norm(cornersBuffer.get(i), cornersProjected, Core.NORM_L2)

            val n = objectPoints[i].rows()
            viewErrors[i] = sqrt(error * error / n).toFloat()
            totalError += error * error
            totalPoints += n
        }
        perViewErrors.create(objectPoints.size, 1, CvType.CV_32FC1)
        perViewErrors.put(0, 0, viewErrors)

        return sqrt(totalError / totalPoints)
    }

    private fun calcBoardCornerPositions(corners: Mat) {
        val cn = 3
        val positions = FloatArray(numCorners * cn)

        for (i in 0 until patternSize.height.toInt()) {
            var j = 0
            while (j < patternSize.width * cn) {
                positions[(i * patternSize.width * cn + j + 0) as Int] =
                    (2 * (j / cn) + i % 2) * squareSize as Float
                positions[(i * patternSize.width * cn + j + 1) as Int] =
                    i * squareSize as Float
                positions[(i * patternSize.width * cn + j + 2) as Int] = 0f
                j += cn
            }
        }
        corners.create(numCorners, 1, CvType.CV_32FC3)
        corners.put(0, 0, positions)
    }

    override fun onDrawFrame(p0: Canvas?, p1: Int, p2: Int, p3: Float, p4: Float, p5: Any?) {
    }
}