import edu.wpi.first.deployutils.deploy.artifact.Artifact
import edu.wpi.first.deployutils.deploy.artifact.FileTreeArtifact
import edu.wpi.first.gradlerio.GradleRIOPlugin
import edu.wpi.first.gradlerio.deploy.roborio.FRCJavaArtifact
import edu.wpi.first.gradlerio.deploy.roborio.RoboRIO
import edu.wpi.first.toolchain.NativePlatforms
import groovy.json.JsonSlurper
import org.gradle.plugins.ide.idea.model.IdeaLanguageLevel
import org.jetbrains.kotlin.gradle.dsl.JvmTarget

plugins {
    java
    idea
    id("org.jetbrains.kotlin.jvm") version "2.0.21"
    id("edu.wpi.first.GradleRIO") version "2024.3.2"
    id("com.peterabeles.gversion") version "1.10"
    id("com.google.devtools.ksp") version "2.0.21-1.0.25"
}

val javaVersion = JavaVersion.VERSION_17

allprojects {
    plugins.apply("java")
    java {
        sourceCompatibility = javaVersion
        targetCompatibility = javaVersion
    }
}

val robotMainClass = "com.frcteam3636.frc2024.Main"

repositories {
    maven {
        url = uri("https://maven.pkg.github.com/Mechanical-Advantage/AdvantageKit")
        credentials {
            username = "Mechanical-Advantage-Bot"
            password = "\u0067\u0068\u0070\u005f\u006e\u0056\u0051\u006a\u0055\u004f\u004c\u0061\u0079\u0066\u006e\u0078\u006e\u0037\u0051\u0049\u0054\u0042\u0032\u004c\u004a\u006d\u0055\u0070\u0073\u0031\u006d\u0037\u004c\u005a\u0030\u0076\u0062\u0070\u0063\u0051"
        }
    }
    mavenLocal()
}

configurations.configureEach {
    exclude("edu.wpi.first.wpilibj")
}

tasks.register<JavaExec>("checkAkitInstall") {
    dependsOn("classes")
    mainClass = "org.littletonrobotics.junction.CheckInstall"
    classpath = sourceSets.main.get().runtimeClasspath
}
tasks.compileJava.map { it.finalizedBy(tasks["checkAkitInstall"]) }

gversion {
    srcDir = "src/main/java/"
    classPackage = ""
    className = "BuildConstants"
    dateFormat = "yyyy-MM-dd HH:mm:ss z"
    timeZone = "America/Los_Angeles" // Use preferred time zone
    indent = "    "
}
tasks.assemble.map { it.dependsOn(tasks["createVersionFile"]) }

// Define my targets (RoboRIO) and artifacts (deployable files)
// This is added by GradleRIO's backing project DeployUtils.
deploy {
    targets {
        register<RoboRIO>("roborio") {
            // Team number is loaded either from the .wpilib/wpilib_preferences.json
            // or from command line. If not found an exception will be thrown.
            // You can use getTeamOrDefault(team) instead of getTeamNumber if you
            // want to store a team number in this file.
            team = project.frc.getTeamNumber()
            debug = project.frc.getDebugOrDefault(false)

            this.artifacts {
                register<FRCJavaArtifact>("frcJava") {
                    jvmArgs.add("-ea") // Remove this flag during comp to disable asserts
                    dependsOn(tasks.jar.get())
                    setJarTask(tasks.jar.get())
                }

                // publish files from `deploy/`
                register<FileTreeArtifact>("frcStaticFileDeploy") {
                    files = project.fileTree("src/main/deploy")
                    directory = "/home/lvuser/deploy"
                }
            }
        }
    }
}

// Set to true to use debug for JNI.
wpi.java.debugJni = false

dependencies {
    wpi.java.deps.wpilib().forEach { implementation(it) }
    wpi.java.vendor.java().forEach { implementation(it) }

    implementation(project(":annotation"))
    ksp(project(":annotation"))

    wpi.java.deps.wpilibJniDebug(NativePlatforms.roborio).forEach {
        "roborioDebug"(it)
        nativeDebug(it)
    }
    wpi.java.vendor.jniDebug(NativePlatforms.roborio).forEach {
        "roborioDebug"(it)
        nativeDebug(it)
    }

    wpi.java.deps.wpilibJniRelease(NativePlatforms.roborio).forEach {
        "roborioRelease"(it)
        nativeRelease(it)
    }
    wpi.java.vendor.jniRelease(NativePlatforms.roborio).forEach {
        "roborioRelease"(it)
        nativeRelease(it)
    }

    wpi.sim.enableDebug().forEach { simulationDebug(it) }
    wpi.sim.enableRelease().forEach { simulationRelease(it) }

    implementation("org.jetbrains.kotlin:kotlin-stdlib")
}

// Simulation configuration (e.g. environment variables).
wpi.sim.addGui().defaultEnabled = true
wpi.sim.addDriverstation()

// Setting up my Jar File. In this case, adding all libraries into the main jar ('fat jar')
// in order to make them all available at runtime. Also adding the manifest so WPILib
// knows where to look for our Robot Class.
tasks {
    jar {
        group = "build"
        dependsOn(configurations.runtimeClasspath)
        from(sourceSets.main.get().allSource)
        from(
            configurations.runtimeClasspath.get().map {
                if(it.isDirectory()) { it } else { zipTree(it) }
            }
        )
        manifest(GradleRIOPlugin.javaManifest(robotMainClass))
        duplicatesStrategy = DuplicatesStrategy.INCLUDE
    }
}

// Configure jar and deploy tasks
wpi.java.configureExecutableTasks(tasks.jar.get())
wpi.java.configureTestTasks(tasks.test.get())

tasks.withType(JavaCompile::class).configureEach {
    // Configure string concat to always inline compile
    options.compilerArgs.add("-XDstringConcat=inline")
}

kotlin {
    compilerOptions {
        // https://kotlinlang.org/docs/gradle-configure-project.html#gradle-java-toolchains-support
        jvmTarget.set(JvmTarget.JVM_17)
        jvmToolchain(17)
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
