package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.Component
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.rad
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
            var averagePose = Pose2d()
            if (detections.isNotEmpty()) {
                detections.forEach {
                    averagePose += Pose2d(it.ftcPose.x, it.ftcPose.y, it.ftcPose.yaw.rad)
                }
                averagePose /= detections.size.toDouble()
            }
            telem.drawRobot(averagePose, "blue")
            telem.addData("pose", averagePose)
        }
        initLoop = true
    }
}