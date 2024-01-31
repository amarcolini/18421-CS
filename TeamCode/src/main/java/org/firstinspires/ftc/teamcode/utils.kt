package org.firstinspires.ftc.teamcode

import com.amarcolini.joos.command.Command
import com.amarcolini.joos.command.SequentialCommand
import com.amarcolini.joos.command.WaitCommand
import com.amarcolini.joos.geometry.Angle
import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.geometry.Vector2d
import com.amarcolini.joos.hardware.drive.DrivePathFollower
import com.amarcolini.joos.path.PathBuilder
import com.amarcolini.joos.path.PathContinuityViolationException
import com.amarcolini.joos.path.heading.*

const val tile = 24.0

class PathCommandBuilder(
    private val drive: DrivePathFollower,
    startPose: Pose2d,
    startTangent: Angle = startPose.heading,
) {
    private var builder = PathBuilder(startPose, startTangent)

    private var currentCommand = SequentialCommand()

    private fun tryAdd(segment: PathBuilder.() -> Unit): PathCommandBuilder {
        try {
            builder.segment()
        } catch (e: PathContinuityViolationException) {
            pushPath()
            builder.segment()
        }
        return this
    }

    private fun pushPath(newTangent: Angle? = null): Command? {
        val path = builder.build()
        builder = PathBuilder(
            path.end(),
            newTangent?.let { Pose2d(it.vec()) } ?: path.endDeriv(),
            path.endSecondDeriv()
        )
        return if (path.segments.isNotEmpty()) drive.followerPath(path)
        else null
    }

    private fun pushAndAddPath(newTangent: Angle? = null) {
        pushPath(newTangent)?.let {
            currentCommand = currentCommand then it
        }
    }

    fun then(command: Command): PathCommandBuilder {
        pushAndAddPath()
        currentCommand = currentCommand then command
        return this
    }

    fun and(command: Command): PathCommandBuilder {
        val original = pushPath()
        currentCommand = currentCommand then (
                if (original != null) (original and command)
                else command
                )
        return this
    }

    fun wait(duration: Double) = then(WaitCommand(duration))

    fun build() = currentCommand

    /**
     * Adds a line segment with the specified heading interpolation.
     *
     * @param endPosition end position
     * @param headingInterpolation desired heading interpolation
     * @see HeadingInterpolation
     */
    fun addLine(endPosition: Vector2d, headingInterpolation: HeadingInterpolation = TangentHeading) =
        tryAdd {
            this.addLine(endPosition, headingInterpolation)
        }

    fun lineTo(endPosition: Vector2d) = addLine(endPosition, TangentHeading)
    fun lineToConstantHeading(endPosition: Vector2d) = addLine(endPosition, ConstantHeading)
    fun lineToLinearHeading(endPose: Pose2d) = addLine(endPose.vec(), LinearHeading(endPose.heading))
    fun lineToSplineHeading(endPose: Pose2d) = addLine(endPose.vec(), SplineHeading(endPose.heading))
    fun forward(distance: Double) = tryAdd { forward(distance) }
    fun back(distance: Double) = forward(-distance)
    fun strafeLeft(distance: Double) = tryAdd { strafeLeft(distance) }
    fun strafeRight(distance: Double) = strafeLeft(-distance)

    /**
     * Adds a spline segment with the specified heading interpolation.
     *
     * @param endPosition end position
     * @param endTangent end tangent
     * @param startTangentMag the magnitude of the start tangent (negative = default magnitude)
     * @param endTangentMag the magnitude of the end tangent (negative = default magnitude)
     * @param headingInterpolation desired heading interpolation
     * @see HeadingInterpolation
     */
    @JvmOverloads
    fun addSpline(
        endPosition: Vector2d,
        endTangent: Angle,
        headingInterpolation: HeadingInterpolation = TangentHeading,
        startTangentMag: Double = -1.0,
        endTangentMag: Double = -1.0,
    ) = tryAdd {
        this.addSpline(endPosition, endTangent, startTangentMag, endTangentMag, headingInterpolation)
    }

    fun splineTo(endPosition: Vector2d, endTangent: Angle) = addSpline(endPosition, endTangent)
    fun splineToConstantHeading(endPosition: Vector2d, endTangent: Angle) =
        addSpline(endPosition, endTangent, ConstantHeading)

    fun splineToLinearHeading(endPose: Pose2d, endTangent: Angle) =
        addSpline(endPose.vec(), endTangent, LinearHeading(endPose.heading))

    fun splineToSplineHeading(endPose: Pose2d, endTangent: Angle) =
        addSpline(endPose.vec(), endTangent, SplineHeading(endPose.heading))
}