package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.opencv.core.Mat
import org.openftc.easyopencv.TimestampedOpenCvPipeline

@JoosConfig
class AprilTagPipeline(var calibration: () -> CameraCalibration) : TimestampedOpenCvPipeline() {
    companion object {
        var webcamPose = Pose2d()
    }

    val processor = AprilTagProcessor
        .easyCreateWithDefaults()
//        .Builder()
//        .setDrawTagID(true)
//        .setDrawTagOutline(true)
//        .setDrawAxes(true)
//        .setTagLibrary(getCenterStageTagLibrary())
//        .build()

    override fun init(mat: Mat) {
        processor.init(mat.width(), mat.height(), calibration())
    }

    override fun processFrame(input: Mat, captureTimeNanos: Long): Mat {
        requestViewportDrawHook(
            processor.processFrame(input, captureTimeNanos)
        )
        return input
    }

    fun getPose(tag: AprilTagDetection): Pose2d {
        // Get the tag absolute position on the field
        val tagPose = Transform3d(
            tag.metadata.fieldPosition,
            tag.metadata.fieldOrientation
        )

        // Get the relative location of the tag from the camera
        val cameraToTagTransform = Transform3d(
            VectorF(
                tag.rawPose.x.toFloat(),
                tag.rawPose.y.toFloat(),
                tag.rawPose.z.toFloat()
            ),
            Transform3d.MatrixToQuaternion(tag.rawPose.R)
        )

        // Inverse the previous transform to get the location of the camera from the tag
        val tagToCameraTransform = cameraToTagTransform.unaryMinusInverse()

        // Add the tag position and the relative position of the camera to the tag
        val cameraPose = tagPose.plus(tagToCameraTransform)

        // The the relative location of the camera to the robot
        //TODO: You have to tune this value for your camera
        val robotToCameraTransform = Transform3d(
            VectorF(
                webcamPose.x.toFloat(),
                webcamPose.y.toFloat(),
                8.50f
            ),
            Quaternion(
                webcamPose.heading.div(2.0).cos().toFloat(),
                0f,
                webcamPose.heading.div(2.0).sin().toFloat(),
                0f,
                System.nanoTime()
            ),
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

        // Convert from a 3D transform to a 2D pose
        return robotPose.toPose2d()
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