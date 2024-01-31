package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibrationHelper
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation


@TeleOp
class AprilTagTest : CommandOpMode() {
    private val robot by robot<CSRobot>()
    val pipeline = AprilTagPipeline {
        CameraCalibrationHelper.getInstance().getCalibration(robot.webcam.calibrationIdentity, 640, 480)
    }

    override fun preInit() {
        robot.webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                robot.webcam.startStreaming(
                    640,
                    480,
                    OpenCvCameraRotation.UPRIGHT
                )
                robot.webcam.setPipeline(pipeline)
                FtcDashboard.getInstance().startCameraStream(robot.webcam, 0.0)
            }

            override fun onError(errorCode: Int) {
                telem.addLine("Camera failed to open!!").setRetained(true)
                telem.update()
                requestOpModeStop()
            }

        })
        schedule(true) {
            val detections = pipeline.processor.detections
            detections.forEachIndexed { _, it ->
                val pose = pipeline.getPose(it)
                if (pose != null) {
                    val i = it.id
                    telem.addLine("p")
                    telem.drawRobot(pose.toPose2d(), "blue")
                    telem.addLine("pose $i: ${pose.translation.data.toList()}")
                    telem.addLine("heading $i: ${pose.zRotation}")
                    telem.addLine("quat $i: ${pose.rotation}")
//                    telem.addLine(
//                        "center $i: ${
//                            Vector2d(
//                                it.center.x, it.center.y
//                            )
//                        }"
//                    )
                }
            }
        }

        initLoop = true
    }
}