package org.firstinspires.ftc.teamcode.opmode.auto

import com.amarcolini.joos.command.CommandOpMode
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.dashboard.JoosConfig
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import org.firstinspires.ftc.teamcode.CSRobot
import org.firstinspires.ftc.teamcode.tile
import org.firstinspires.ftc.teamcode.vision.PropPipeline

@Autonomous
@JoosConfig
class RedClose2p0 : CommandOpMode() {
    private val robot by robot<CSRobot>()
    private val pipeline = PropPipeline(false)

    companion object {
        var startPose = Pose2d(16.0, -3 * tile + 9.0, (90).deg)
        var leftPlopPose = Pose2d(10.0, -35.0, (-180).deg)
        var leftPlacePose = Pose2d(50.0, -30.0, 0.deg)
        var centerPlopPose = Pose2d(16.0, -40.0, (90).deg)
        var centerPlacePose = Pose2d(50.0, -36.0, 0.deg)
        var rightPlopPose = Pose2d(23.0, -42.0, (90).deg)
        var rightPlacePose = Pose2d(50.0, -44.0, 0.deg)
        var parkPose = Pose2d(52.0, -64.0, 0.deg)
    }

    override fun preInit() {
        robot.outtake.init()
        robot.pixelPlopper.prime()
        robot.openFrontCameraAsync {
            robot.frontCamera.setPipeline(pipeline)
            schedule(true) {
                telem.addData("prop location", pipeline.lastKnownLocation)
            }
        }
    }

    override fun preStart() {
        cancelAll()
        robot.outtake.ready()
        robot.drive.poseEstimate = startPose

        val (plopPose, placePose) = when (pipeline.lastKnownLocation) {
            PropPipeline.PropLocation.Left -> leftPlopPose to leftPlacePose
            PropPipeline.PropLocation.Center -> centerPlopPose to centerPlacePose
            PropPipeline.PropLocation.Right -> rightPlopPose to rightPlacePose
        }
        val purplePlopCommand = robot.drive.pathCommandBuilder(startPose)
            .lineToSplineHeading(plopPose)
            .then(robot.pixelPlopper.plop())
            .build()
        val yellowPlaceCommand = robot.drive.pathCommandBuilder(plopPose)
            .back(3.0)
            .lineToSplineHeading(placePose)
            .wait(0.1)
            .forward(3.0).withTimeout(1.5)
            .then(robot.outtake.extend())
            .wait(2.0)
            .then(robot.outtake::releaseRight)
            .wait(1.0)
            .then(robot.outtake.reset())
            .back(3.0)
            .lineToSplineHeading(parkPose)
            .build()

        SequentialCommand(
            true,
            purplePlopCommand,
            yellowPlaceCommand,
        ).thenStopOpMode().schedule()
    }
}