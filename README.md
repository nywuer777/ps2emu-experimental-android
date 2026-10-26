# PS2 Experimental Android

ARM64 Android diagnostic frontend for the experimental Rust PS2 core. It imports a user-provided BIOS image and ISO through Android's file picker, loads the BIOS into the EE and IOP maps, resolves `SYSTEM.CNF`, loads its boot ELF directly into EE RAM, mounts the ISO as IOP CD/DVD media, and runs a bounded instruction probe. The ELF path bypasses BIOS startup; this package is not a verified game emulator and does not claim retail-game compatibility.

The GitHub Actions workflow builds `app-debug.apk` for arm64-v8a and publishes it as the `ps2emu-diagnostic-apk` artifact. Open Actions, choose **Build Android diagnostic APK**, run the workflow, then download the artifact from the completed run.
