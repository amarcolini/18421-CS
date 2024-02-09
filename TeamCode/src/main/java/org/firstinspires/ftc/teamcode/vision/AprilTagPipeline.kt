package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.*
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.Transform3d
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCenterStageTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.opencv.core.Mat
import org.openftc.easyopencv.TimestampedOpenCvPipeline
import kotlin.math.cos
import kotlin.math.sin

@JoosConfig
class AprilTagPipeline(var calibration: () -> CameraCalibration) : TimestampedOpenCvPipeline() {
    companion object {
        var webcamPose = Pose2d()
    }

    val processor = AprilTagProcessor
//        .easyCreateWithDefaults()
        .Builder()
        .setDrawTagID(true)
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setTagLibrary(getCenterStageTagLibrary())
        .setLensIntrinsics(926.725118, 940.8540229, 443.3417136, 310.9205909)
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

    fun getPose(tag: AprilTagDetection): Transform3d? {
        if (tag.metadata == null) return null

        // Get the tag absolute position on the field
        val tagPose = Transform3d(
            tag.metadata.fieldPosition,
            tag.metadata.fieldOrientation
        )

        // Get the relative location of the tag from the camera
        val orientation =
            Orientation.getOrientation(tag.rawPose.R, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS)
        val xQ = Quaternion(
            cos(orientation.thirdAngle), sin(orientation.thirdAngle), 0f, 0f, 0
        )
        val yQ = Quaternion(
            cos(-orientation.firstAngle), 0f, sin(-orientation.firstAngle), 0f, 0
        )
        val zQ = Quaternion(
            cos(-orientation.secondAngle), 0f, 0f, sin(-orientation.secondAngle), 0
        )

        val cameraToTagTransform = Transform3d(
            VectorF(
                tag.rawPose.x.toFloat(),
                tag.rawPose.y.toFloat(),
                tag.rawPose.z.toFloat(),
            ),
//        xQ.multiply(yQ, 0).multiply(zQ, 0),
            Transform3d.matrixToQuaternion(tag.rawPose.R)
        )

        // Inverse the previous transform to get the location of the camera from the tag
        val tagToCameraTransform = cameraToTagTransform.unaryMinusInverse()

        // Add the tag position and the relative position of the camera to the tag
        val cameraPose = tagPose.plus(tagToCameraTransform)

        // The the relative location of the camera to the robot
        //TODO: You have to tune this value for your camera
        val robotToCameraTransform = Transform3d(
            VectorF(
                17.229937f,
                1.5510408f,
                -9.339169f
            ),
            Quaternion(0.539f, 0.026f, -0.063f, 0.839f, System.nanoTime())
//        Quaternion.identityQuaternion(),
//            Transform3d.MatrixToQuaternion(
//                Orientation.getRotationMatrix(
//                    AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS,
//                    webcamPose.heading.radians.toFloat(), 0f, 0f
//                )
//            )
        )

        // Inverse the previous transform again to get the location of the robot from the camera
        val cameraToRobotTransform = robotToCameraTransform.unaryMinusInverse()

        // Add the relative location of the robot to location of the Camera
        val robotPose = cameraPose.plus(cameraToRobotTransform)

//        robotPose.plus(cameraPose.unaryMinusInverse()).unaryMinusInverse()

        // Convert from a 3D transform to a 2D pose
        return cameraPose
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