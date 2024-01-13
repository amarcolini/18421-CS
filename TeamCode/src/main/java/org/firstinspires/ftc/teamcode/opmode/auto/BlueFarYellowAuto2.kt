package org.firstinspires.ftc.teamcode.opmode.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.path.PathBuilder
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.tile
import org.firstinspires.ftc.teamcode.vision.PropPipeline
import org.openftc.easyopencv.OpenCvCamera
import org.openftc.easyopencv.OpenCvCameraRotation

@Autonomous
@JoosConfig
class BlueFarYellowAuto2 : CommandOpMode() {
    private val robot by robot<CSRobot>()
    private val pipeline = PropPipeline(false)

    companion object {
        var startPose = Pose2d(-16.0, 3 * tile - 9.0, (-90).deg)
        var rightPlopPose = Pose2d(-15.5, 33.0, (-180).deg)
        var centerPlopPose = Pose2d(-25.0, 28.0, (0).deg)
        var leftPlopPose = Pose2d(-10.0, 33.0, (0).deg)
        var rightPlacePose = Pose2d(71.5, 37.0, 5.deg)
        var centerPlacePose = Pose2d(71.5, 40.0, 0.deg)
        var leftPlacePose = Pose2d(71.5, 48.0, 0.deg)
        var regularExitPose = Pose2d(-16.0, 13.0, 0.deg)
        var centerExitPose = Pose2d(-26.0, 13.0, 0.deg)
        var crossPose = Pose2d(62.0, 17.0, 0.deg)
    }

    override fun preInit() {
        robot.outtake.init()
        robot.pixelPlopper.prime()
        robot.webcam.openCameraDeviceAsync(object : OpenCvCamera.AsyncCameraOpenListener {
            override fun onOpened() {
                robot.webcam.startStreaming(
                    640,
                    480,
                    OpenCvCameraRotation.UPRIGHT
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
        robot.drive.poseEstimate = startPose
        val location = pipeline.lastKnownLocation

        val (plopPose, placePose) = when (location) {
            PropPipeline.PropLocation.Left -> leftPlopPose to leftPlacePose
            PropPipeline.PropLocation.Center -> centerPlopPose to centerPlacePose
            PropPipeline.PropLocation.Right -> rightPlopPose to rightPlacePose
        }

        val purplePlopPath = PathBuilder(startPose)
            .lineToSplineHeading(plopPose)
            .build()
        val purpleBackUpTrajectory = PathBuilder(purplePlopPath.end())
            .back(3.0)
            .build()

//        val yellowPlaceTrajectory =
//            robot.drive.trajectoryBuilder(purplePlopTrajectory.end())
//                .back(3.0)
//                .lineToSplineHeading(placePose)
//                .build()
        val exitPaths = listOf(
            purpleBackUpTrajectory.end(),
            if (location == PropPipeline.PropLocation.Center) {
                centerExitPose
            } else regularExitPose,
            crossPose, placePose
        ).zipWithNext { a, b ->
            PathBuilder(a)
                .lineToSplineHeading(b).build()
        }

//        val exitPath = PathBuilder(purpleBackUpTrajectory.end())
//            .run {
//                if (location == PropPipeline.PropLocation.Center) {
//                    lineToSplineHeading(centerExitPose)
//                } else lineToSplineHeading(regularExitPose)
//                lineToSplineHeading(crossPose)
//                lineToSplineHeading(placePose)
//                build()
//            }

        val purplePlopCommand = robot.drive.followerPath(purplePlopPath)
            .then(robot.pixelPlopper.plop())
            .then(robot.drive.followerPath(purpleBackUpTrajectory))

        val yellowPlaceCommand = SequentialCommand(
            *exitPaths.map { robot.drive.followerPath(it) }.toTypedArray()
        )
            .then(robot.outtake.extend())
            .wait(2.0)
            .then(robot.outtake::releaseRight)
            .wait(2.0)
            .then(robot.outtake.reset())
            .wait(2.0)

        SequentialCommand(
            true,
            robot.outtake.ready(),
            purplePlopCommand,
            yellowPlaceCommand
        ).thenStopOpMode().schedule()
    }
}