package org.firstinspires.ftc.teamcode.vision

import com.amarcolini.joos.geometry.Pose2d
import com.amarcolini.joos.util.deg
import com.amarcolini.joos.util.rad
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF
import org.firstinspires.ftc.robotcore.external.matrices.VectorF
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion
import kotlin.math.atan2
import kotlin.math.sqrt


/**
 * Class for transformations in 3D Space. Only uses libraries in the FTC SDK,
 * with the optional addition of RoadRunner for ease of use with relocalization.
 */
class Transform3d {
    /**
     * Vector to describe a 3-dimensional position in space
     */
    var translation: VectorF

    /**
     * Quaternion to describe a 3-dimensional orientation in space
     */
    var rotation: Quaternion

    constructor() {
        translation = VectorF(0f, 0f, 0f)
        rotation = Quaternion()
    }

    constructor(t: VectorF, Q: Quaternion) {
        translation = t
        rotation = Q
    }

    fun unaryMinusInverse(): Transform3d {
        val Q = rotation.inverse()
        val t = rotation.applyToVector(zeroVector.subtracted(translation))
        return Transform3d(t, Q)
    }

    fun plus(P: Transform3d): Transform3d {
        val t = translation.added(rotation.applyToVector(P.translation))
        val Q = rotation.multiply(P.rotation, System.nanoTime())
        return Transform3d(t, Q)
    }

    val z: Double
        get() = atan2(
            2.0 * (rotation.y * rotation.z + rotation.w * rotation.x),
            (rotation.w * rotation.w - rotation.x * rotation.x - rotation.y * rotation.y + rotation.z * rotation.z).toDouble()
        )

    fun toPose2d(): Pose2d {
        return Pose2d(
            translation[0].toDouble(),
            translation[1].toDouble(),
            this.z.rad
        )
    }

    companion object {
        private val zeroVector = VectorF(0f, 0f, 0f)

        fun MatrixToQuaternion(m1: MatrixF): Quaternion {
            val w = (sqrt(1.0 + m1[0, 0] + m1[1, 1] + m1[2, 2]) / 2.0).toFloat()
            val w4 = (4.0 * w).toFloat()
            val x = (m1[2, 1] - m1[1, 2]) / w4
            val y = (m1[0, 2] - m1[2, 0]) / w4
            val z = (m1[1, 0] - m1[0, 1]) / w4
            return Quaternion(w, x, y, z, System.nanoTime())
        }
    }
}