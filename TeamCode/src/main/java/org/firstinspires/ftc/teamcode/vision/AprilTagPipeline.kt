package org.firstinspires.ftc.teamcode.vision

import android.graphics.Canvas
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.robotcore.external.navigation.Orientation
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration
import org.firstinspires.ftc.teamcode.Transform3d
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase.getCenterStageTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor
import org.opencv.core.Mat
import org.openftc.easyopencv.TimestampedOpenCvPipeline


@JoosConfig
class AprilTagPipeline(var calibration: () -> CameraCalibration) : TimestampedOpenCvPipeline() {
    companion object {
        var webcamPose = Pose2d(
            3.71, -1.91, (-93.3).deg
        )
    }

    private val customLibrary = AprilTagLibrary.Builder()
        .addTag(
            1, "BlueAllianceLeft",
            2.0, VectorF(61.75f, 41.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            2, "BlueAllianceCenter",
            2.0, VectorF(61.75f, 35.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            3, "BlueAllianceRight",
            2.0, VectorF(61.75f, 29.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            4, "RedAllianceLeft",
            2.0, VectorF(61.75f, -29.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            5, "RedAllianceCenter",
            2.0, VectorF(61.75f, -35.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            6, "RedAllianceRight",
            2.0, VectorF(61.75f, -41.41f, 4f), DistanceUnit.INCH,
            Quaternion(0.3536f, -0.6124f, 0.6124f, -0.3536f, 0)
        )
        .addTag(
            7, "RedAudienceWallLarge",
            5.0, VectorF(-70.25f, -40.625f, 5.5f), DistanceUnit.INCH,
            Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0)
        )
        .addTag(
            8, "RedAudienceWallSmall",
            2.0, VectorF(-70.25f, -35.125f, 4f), DistanceUnit.INCH,
            Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0)
        )
        .addTag(
            9, "BlueAudienceWallSmall",
            2.0, VectorF(-70.25f, 35.125f, 4f), DistanceUnit.INCH,
            Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0)
        )
        .addTag(
            10, "BlueAudienceWallLarge",
            5.0, VectorF(-70.25f, 40.625f, 5.5f), DistanceUnit.INCH,
            Quaternion(0.5f, -0.5f, -0.5f, 0.5f, 0)
        )
        .build()

    val processor = AprilTagProcessor
//        .easyCreateWithDefaults()
        .Builder()
        .setDrawTagID(true)
        .setDrawTagOutline(true)
        .setDrawAxes(true)
        .setTagLibrary(customLibrary)
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

    fun getPose(tag: AprilTagDetection): Pose2d? {
        if (tag.metadata == null) return null

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
        //36.17
        //-37.32
        //0.deg

        //39.88
        //-39.23
        //-93.3
        val robotToCameraTransform = Transform3d(
            VectorF(
                webcamPose.x.toFloat(),
                webcamPose.y.toFloat(),
                0f,
            ),
//            Quaternion(0.539f, 0.026f, -0.063f, 0.839f, System.nanoTime())
//        Quaternion.identityQuaternion(),
            Transform3d.matrixToQuaternion(
                Orientation.getRotationMatrix(
                    AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES,
                    webcamPose.heading.degrees.toFloat(), 0f, 0f
                )
            )
        )

        // Inverse the previous transform again to get the location of the robot from the camera
        val cameraToRobotTransform = robotToCameraTransform.unaryMinusInverse()

        val initialPose = cameraPose.toPose2d()
        val newHeading = initialPose.heading - webcamPose.heading
        val robotPose = Pose2d(
            initialPose.vec() - webcamPose.vec().rotated(newHeading),
            newHeading
        )

        // Add the relative location of the robot to location of the Camera
//        val robotPose = cameraPose.plus(cameraToRobotTransform)

//        robotPose.plus(cameraPose.unaryMinusInverse()).unaryMinusInverse()

        // Convert from a 3D transform to a 2D pose
        return robotPose
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