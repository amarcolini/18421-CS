import com.google.devtools.ksp.gradle.KspTask
import org.team11260.FastLoadPlugin
import org.team11260.tasks.ReloadFastLoad

//
// build.gradle in TeamCode
//
// Most of the definitions for building your module reside in a common, shared
// file "build.common.gradle". Being factored in this way makes it easier to
// integrate updates to the FTC into your code. If you really need to customize
// the build definitions, you can place those customizations in this file, but
// please think carefully as to whether such customizations are really necessary
// before doing so.


// Custom definitions may go here
buildscript {
    repositories {
        mavenCentral()
        maven {
            url = uri("https://www.matthewo.tech/maven/")
        }
    }
    dependencies {
        classpath("org.team11260:fast-load-plugin:0.1.2")
    }
}

plugins {
    id("com.google.devtools.ksp") version "1.8.20-1.0.11"
    id("com.android.application")
    id("org.jetbrains.kotlin.android") version "1.8.20"
}

repositories {
//    maven { url = uri("C:/Users/Alessandro/IdeaProjects/joos/testRepo") }
    maven { url = uri("https://jitpack.io") }
    maven { url = uri("https://maven.brott.dev/") }
    maven {
        url = uri("https://www.matthewo.tech/maven/")
    }
}

// Include common definitions from above.
apply(from = "../build.common.gradle")
apply(from = "../build.common.gradle")
apply(from = "../build.dependencies.gradle")
apply(plugin = "org.team11260.fast-load-plugin")

android {
    namespace = "org.firstinspires.ftc.teamcode"

    packagingOptions {
        jniLibs.useLegacyPackaging = true
    }
}

tasks.withType<KspTask> {
    onlyIf {
        !gradle.startParameter.taskNames.contains("reloadFastLoad")
    }
}

dependencies {
    implementation(project(":FtcRobotController"))
    annotationProcessor(files("lib/OpModeAnnotationProcessor.jar"))

    implementation("com.github.amarcolini.joos:command:0.4.9")
    ksp("com.github.amarcolini.joos:annotation:0.4.9")
    implementation("org.team11260:fast-load:0.1.2")
//    implementation "org.jetbrains.kotlin:kotlin-reflect:1.9.20"
}