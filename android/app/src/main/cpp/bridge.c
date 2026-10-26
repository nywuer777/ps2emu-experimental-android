#include <jni.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
extern void *ps2emu_create(void);
extern void ps2emu_destroy(void *);
extern int32_t ps2emu_load_bios(void *, const char *);
extern int32_t ps2emu_load_disc(void *, const char *);
extern int32_t ps2emu_run(void *, uint64_t);
extern size_t ps2emu_status(const void *, char *, size_t);
JNIEXPORT jlong JNICALL Java_org_horizon_ps2_MainActivity_nativeCreate(JNIEnv *e,jobject o){(void)e;(void)o;return (jlong)(intptr_t)ps2emu_create();}
JNIEXPORT void JNICALL Java_org_horizon_ps2_MainActivity_nativeDestroy(JNIEnv *e,jobject o,jlong h){(void)e;(void)o;ps2emu_destroy((void*)(intptr_t)h);}
static int call_path(JNIEnv*e,jlong h,jstring p,int disc){if(!p)return -1;const char*s=(*e)->GetStringUTFChars(e,p,0);if(!s)return -2;int r=disc?ps2emu_load_disc((void*)(intptr_t)h,s):ps2emu_load_bios((void*)(intptr_t)h,s);(*e)->ReleaseStringUTFChars(e,p,s);return r;}
JNIEXPORT jint JNICALL Java_org_horizon_ps2_MainActivity_nativeLoadBios(JNIEnv*e,jobject o,jlong h,jstring p){(void)o;return call_path(e,h,p,0);}
JNIEXPORT jint JNICALL Java_org_horizon_ps2_MainActivity_nativeLoadDisc(JNIEnv*e,jobject o,jlong h,jstring p){(void)o;return call_path(e,h,p,1);}
JNIEXPORT jint JNICALL Java_org_horizon_ps2_MainActivity_nativeRun(JNIEnv*e,jobject o,jlong h,jlong n){(void)e;(void)o;return ps2emu_run((void*)(intptr_t)h,(uint64_t)n);}
JNIEXPORT jstring JNICALL Java_org_horizon_ps2_MainActivity_nativeStatus(JNIEnv*e,jobject o,jlong h){(void)o;char b[512]={0};ps2emu_status((void*)(intptr_t)h,b,sizeof b);return (*e)->NewStringUTF(e,b);}
