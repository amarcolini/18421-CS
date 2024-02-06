package org.firstinspires.ftc.teamcode.opmode

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.vision.PropPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam
import java.lang.IllegalStateException

@TeleOp(group = "camera")
@JoosConfig
class CameraTest : CommandOpMode() {
    private val robot by robot<CSRobot>()

    private val pipeline = PropPipeline(mirrored)

    companion object {
        var mirrored = false
    }

    override fun preInit() {
        robot.frontCamera.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                robot.frontCamera.startStreaming(
                    640,
                    480,
                    OpenCvCameraRotation.UPRIGHT,
                    OpenCvWebcam.StreamFormat.MJPEG
                )
                robot.frontCamera.setPipeline(pipeline)
                FtcDashboard.getInstance().startCameraStream(robot.frontCamera, 0.0)

                schedule(true) {
                    telem.addData("prop location", pipeline.lastKnownLocation)
                }
            }

            override fun onError(errorCode: Int) {
                throw IllegalStateException("Camera no open :(")
            }
        })
        initLoop = true
    }
}