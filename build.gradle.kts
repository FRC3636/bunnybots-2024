import edu.wpi.first.deployutils.deploy.artifact.FileTreeArtifact
import edu.wpi.first.gradlerio.GradleRIOPlugin
import edu.wpi.first.gradlerio.deploy.roborio.FRCJavaArtifact
import edu.wpi.first.gradlerio.deploy.roborio.RoboRIO
import edu.wpi.first.toolchain.NativePlatforms
import org.gradle.plugins.ide.idea.model.IdeaLanguageLevel
import org.jetbrains.kotlin.gradle.dsl.JvmTarget

plugins {
    java
    idea
    kotlin("jvm")
    id("com.google.devtools.ksp")
    id("edu.wpi.first.GradleRIO") version "2024.3.2"
    id("com.peterabeles.gversion") version "1.10"
}

val javaVersion = 17
val robotMainClass = "com.frcteam3636.frc2024.Main"

allprojects {
    plugins.apply("java")
    plugins.apply("org.jetbrains.kotlin.jvm")

    java {
        val version = JavaVersion.toVersion(javaVersion)
        sourceCompatibility = version
        targetCompatibility = version
    }

    kotlin {
        compilerOptions {
            // https://kotlinlang.org/docs/gradle-configure-project.html#gradle-java-toolchains-support
            jvmTarget = JvmTarget.fromTarget(javaVersion.toString())
            jvmToolchain(javaVersion)
        }
    }

    repositories {
        maven {
            url = uri("https://maven.pkg.github.com/Mechanical-Advantage/AdvantageKit")
            credentials {
                username = "Mechanical-Advantage-Bot"
                password =
                    "\u0067\u0068\u0070\u005f\u006e\u0056\u0051\u006a\u0055\u004f\u004c\u0061\u0079\u0066\u006e\u0078\u006e\u0037\u0051\u0049\u0054\u0042\u0032\u004c\u004a\u006d\u0055\u0070\u0073\u0031\u006d\u0037\u004c\u005a\u0030\u0076\u0062\u0070\u0063\u0051"
            }
        }
        mavenLocal()
        mavenCentral()
    }
}

gversion {
    srcDir = layout.buildDirectory.dir("generated/gversion/main/kotlin").get().toString()
    language = "kotlin"
    classPackage = "com.frcteam3636.version"
    dateFormat = "yyyy-MM-dd HH:mm:ss z"
    timeZone = "America/Los_Angeles" // Use preferred time zone
    indent = "    "
}

// Define the targets (RoboRIO) and artifacts (deployable files)
// This is added by GradleRIO's backing project DeployUtils.
deploy {
    targets {
        val roborio by register<RoboRIO>("roborio") {
            // Team number is loaded either from the .wpilib/wpilib_preferences.json
            // or from command line. If not found an exception will be thrown.
            team = frc.getTeamOrDefault(3636)
            debug = frc.getDebugOrDefault(false)


        }

        roborio.artifacts {
            register<FRCJavaArtifact>("frcJava") {
                jvmArgs.add("-ea") // Remove this flag during comp to disable asserts
//                jvmArgs.add("-Dcom.sun.management.jmxremote=true")
//                jvmArgs.add("-Dcom.sun.management.jmxremote.port=1198")
//                jvmArgs.add("-Dcom.sun.management.jmxremote.local.only=false")
//                jvmArgs.add("-Dcom.sun.management.jmxremote.ssl=false")
//                jvmArgs.add("-Dcom.sun.management.jmxremote.authenticate=false")
//                jvmArgs.add("-Djava.rmi.server.hostname=10.36.36.2") // Replace TE.AM with team number
                setJarTask(tasks.jar)
                dependsOn(tasks.assemble)
            }

            register<FileTreeArtifact>("frcStaticFileDeploy") {
                files = project.fileTree("src/main/deploy")
                directory = "/home/lvuser/deploy"
            }
        }
    }
}

configurations.configureEach {
    exclude(group = "edu.wpi.first.wpilibj")
}

wpi {
    with(java) {
        // Set to true to use debug for JNI.
        debugJni = false

        // Configure jar and deploy tasks
        configureExecutableTasks(tasks.jar.get())
        configureTestTasks(tasks.test.get())
    }

    // Simulation configuration (e.g. environment variables).
    with(sim) {
        addGui().defaultEnabled = true
        addDriverstation().defaultEnabled = true
    }
}


dependencies {
    wpi.java.deps.wpilib().forEach { implementation(it) }
    wpi.java.vendor.java().forEach { implementation(it) }

    wpi.java.deps.wpilibJniDebug(NativePlatforms.roborio).forEach { "roborioDebug"(it) }
    wpi.java.vendor.jniDebug(NativePlatforms.roborio).forEach { "roborioDebug"(it) }

    wpi.java.deps.wpilibJniRelease(NativePlatforms.roborio).forEach { "roborioRelease"(it) }
    wpi.java.vendor.jniRelease(NativePlatforms.roborio).forEach { "roborioRelease"(it) }

    wpi.java.deps.wpilibJniDebug(NativePlatforms.desktop).forEach { nativeDebug(it) }
    wpi.java.vendor.jniDebug(NativePlatforms.desktop).forEach { nativeDebug(it) }
    wpi.sim.enableDebug().forEach { simulationDebug(it) }

    wpi.java.deps.wpilibJniRelease(NativePlatforms.desktop).forEach { nativeRelease(it) }
    wpi.java.vendor.jniRelease(NativePlatforms.desktop).forEach { nativeRelease(it) }

    wpi.sim.enableRelease().forEach { simulationRelease(it) }

    implementation(project(":annotation"))
    ksp(project(":annotation"))

//    testImplementation(platform("io.kotest:kotest-bom:5.8.0"))
//    testImplementation("io.kotest:kotest-runner-junit5")
//    testImplementation("io.kotest:kotest-assertions-core")
//    testImplementation("io.mockk:mockk:1.13.9")
}

// Set up the JAR File. In this case, adding all libraries into the main jar ('fat jar')
// in order to make them all available at runtime. Also adding the manifest so WPILib
// knows where to look for our Robot Class.
tasks {
    jar {
        group = "build"
        manifest(GradleRIOPlugin.javaManifest(robotMainClass))
        duplicatesStrategy = DuplicatesStrategy.INCLUDE

        from({
            configurations
                .runtimeClasspath
                .get()
                .map { if (it.isDirectory) it else zipTree(it) }
        })
    }

    test {
        useJUnitPlatform()
    }

    // This check is buggy, also it seems to be removed in AdvantageKit 2025
//    val checkAkitInstall by register<JavaExec>("checkAkitInstall") {
//        dependsOn("classes")
//        mainClass = "org.littletonrobotics.junction.CheckInstall"
//        classpath = sourceSets.main.get().runtimeClasspath
//    }

    compileKotlin {
        dependsOn(createVersionFile)
//        finalizedBy(checkAkitInstall)
    }

    register<Exec>("cleanStaticFiles") {
        commandLine = listOf("ssh", "admin@10.36.36.2", "rm", "-rf", "/home/lvuser/deploy")
    }
}

sourceSets {
    main {
        kotlin {
            srcDir(gversion.srcDir)
        }
    }
}

idea {
    project {
        // The project.sourceCompatibility setting is not always picked up, so we set explicitly
        languageLevel = IdeaLanguageLevel(javaVersion)
    }
    module {
        // Improve development & (especially) debugging experience (and IDEA's capabilities) by having libraries' source & javadoc attached
        isDownloadJavadoc = true
        isDownloadSources = true
        // Exclude the .vscode directory from indexing and search
        excludeDirs.add(file(".vscode"))
    }
}
