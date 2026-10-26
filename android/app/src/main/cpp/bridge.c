#include <jni.h>
#include <dlfcn.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

typedef void *(*create_fn)(void);
typedef void (*destroy_fn)(void *);
typedef int32_t (*load_fn)(void *, const char *);
typedef int32_t (*run_fn)(void *, uint64_t);
typedef size_t (*status_fn)(const void *, char *, size_t);

static pthread_once_t core_once = PTHREAD_ONCE_INIT;
static void *core_library;
static create_fn core_create;
static destroy_fn core_destroy;
static load_fn core_load_bios;
static load_fn core_load_disc;
static run_fn core_run;
static status_fn core_status;
static char core_error[256] = "PS2 core is not initialized";

static void load_core_symbols(void) {
    core_library = dlopen("libps2emu_core.so", RTLD_NOW | RTLD_LOCAL);
    if (core_library == NULL) {
        snprintf(core_error, sizeof(core_error), "Could not load PS2 core: %s", dlerror());
        return;
    }

    dlerror();
    core_create = (create_fn)dlsym(core_library, "ps2emu_create");
    core_destroy = (destroy_fn)dlsym(core_library, "ps2emu_destroy");
    core_load_bios = (load_fn)dlsym(core_library, "ps2emu_load_bios");
    core_load_disc = (load_fn)dlsym(core_library, "ps2emu_load_disc");
    core_run = (run_fn)dlsym(core_library, "ps2emu_run");
    core_status = (status_fn)dlsym(core_library, "ps2emu_status");
    const char *error = dlerror();
    if (error != NULL || core_create == NULL || core_destroy == NULL ||
        core_load_bios == NULL || core_load_disc == NULL || core_run == NULL ||
        core_status == NULL) {
        snprintf(core_error, sizeof(core_error), "PS2 core exports are incomplete: %s",
                 error != NULL ? error : "missing symbol");
        dlclose(core_library);
        core_library = NULL;
        core_create = NULL;
        core_destroy = NULL;
        core_load_bios = NULL;
        core_load_disc = NULL;
        core_run = NULL;
        core_status = NULL;
        return;
    }
    core_error[0] = '\0';
}

static int ensure_core(void) {
    pthread_once(&core_once, load_core_symbols);
    return core_library != NULL;
}

JNIEXPORT jlong JNICALL Java_org_horizon_ps2_MainActivity_nativeCreate(JNIEnv *env, jobject self) {
    (void)env;
    (void)self;
    return ensure_core() ? (jlong)(intptr_t)core_create() : 0;
}

JNIEXPORT void JNICALL Java_org_horizon_ps2_MainActivity_nativeDestroy(JNIEnv *env, jobject self, jlong handle) {
    (void)env;
    (void)self;
    if (ensure_core() && handle != 0) core_destroy((void *)(intptr_t)handle);
}

static int call_path(JNIEnv *env, jlong handle, jstring path, int is_disc) {
    if (!ensure_core()) return -100;
    if (path == NULL || handle == 0) return -1;
    const char *utf = (*env)->GetStringUTFChars(env, path, NULL);
    if (utf == NULL) return -2;
    int result = is_disc
        ? core_load_disc((void *)(intptr_t)handle, utf)
        : core_load_bios((void *)(intptr_t)handle, utf);
    (*env)->ReleaseStringUTFChars(env, path, utf);
    return result;
}

JNIEXPORT jint JNICALL Java_org_horizon_ps2_MainActivity_nativeLoadBios(JNIEnv *env, jobject self, jlong handle, jstring path) {
    (void)self;
    return call_path(env, handle, path, 0);
}

JNIEXPORT jint JNICALL Java_org_horizon_ps2_MainActivity_nativeLoadDisc(JNIEnv *env, jobject self, jlong handle, jstring path) {
    (void)self;
    return call_path(env, handle, path, 1);
}

JNIEXPORT jint JNICALL Java_org_horizon_ps2_MainActivity_nativeRun(JNIEnv *env, jobject self, jlong handle, jlong steps) {
    (void)env;
    (void)self;
    if (!ensure_core()) return -100;
    if (handle == 0) return -1;
    return core_run((void *)(intptr_t)handle, (uint64_t)steps);
}

JNIEXPORT jstring JNICALL Java_org_horizon_ps2_MainActivity_nativeStatus(JNIEnv *env, jobject self, jlong handle) {
    (void)self;
    if (!ensure_core()) return (*env)->NewStringUTF(env, core_error);
    if (handle == 0) return (*env)->NewStringUTF(env, "PS2 core initialization failed");
    char buffer[2048] = {0};
    core_status((void *)(intptr_t)handle, buffer, sizeof(buffer));
    return (*env)->NewStringUTF(env, buffer);
}
