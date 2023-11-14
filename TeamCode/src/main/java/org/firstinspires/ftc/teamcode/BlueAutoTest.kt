package org.firstinspires.ftc.teamcode

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.vision.PropPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation
import org.openftc.easyopencv.OpenCvWebcam

@Autonomous
@JoosConfig
class BlueAutoTest : CommandOpMode() {
    private val robot by robot<CSRobot>()
    private val pipeline = PropPipeline()

    companion object {
        var startPose = Pose2d(0.5 * tile, 2.5 * tile, (-90).deg)
        var rightPlacePose = Pose2d(2 * tile, 1 * tile, 0.deg)
        var centerPlacePose = Pose2d(2 * tile, 1.5 * tile, 0.deg)
        var leftPlacePose = Pose2d(2 * tile, 2 * tile, 0.deg)
    }

    private val centerPlaceTrajectory by onInit {
        robot.drive.trajectoryBuilder(startPose)
            .splineTo(centerPlacePose.vec(), centerPlacePose.heading)
            .build()
    }

    override fun preInit() {
        robot.webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                robot.webcam.startStreaming(
                    640,
                    480,
                    OpenCvCameraRotation.UPRIGHT,
                    OpenCvWebcam.StreamFormat.MJPEG
                )
                robot.webcam.setPipeline(pipeline)
                FtcDashboard.getInstance().startCameraStream(robot.webcam, 0.0)

                schedule(true) {
                    telem.addData("prop location", pipeline.lastKnownLocation)
                }
            }

            override fun onError(errorCode: Int) {
                telem.addLine("Camera failed to open!!").setRetained(true)
                telem.update()
                requestOpModeStop()
            }

        })
    }

    override fun preStart() {
        cancelAll()

        robot.drive.followTrajectory(centerPlaceTrajectory)
            .then(robot.outtake::extend)
            .wait(1.0)
            .then(robot.outtake::releaseRight)
            .wait(1.0)
            .then(robot.outtake::reset)
            .wait(1.0)
            .thenStopOpMode()
            .schedule()
    }
}