plugins {
    alias(libs.plugins.kotlin.android)
    id("com.android.library")
}

android {
    namespace = "fr.intuite.sdrbridge"
    compileSdk = 35

    defaultConfig {
        minSdk = 25
        externalNativeBuild {
            cmake {
                cppFlags.addAll(listOf("-frtti", "-fexceptions"))
                arguments.add("-DANDROID_STL=c++_shared")
            }
        }
        ndk { abiFilters += listOf("armeabi-v7a", "arm64-v8a", "x86_64") }
    }

    externalNativeBuild {
        cmake {
            path = file("CMakeLists.txt")
            version = "3.22.1"
        }
    }

    sourceSets {
        getByName("main") {
            java.srcDirs("java")
        }
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
}

dependencies {
    implementation(libs.androidx.core.ktx)
    implementation("com.google.code.gson:gson:2.10.1")
}

