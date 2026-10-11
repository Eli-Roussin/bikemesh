#ifndef UNITY_H
#define UNITY_H

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

int g_testsRun = 0;
int g_testsPassed = 0;
int g_testsFailed = 0;

// Helper macro for reporting failures
#define UNITY_TEST_FAIL(line, msg) fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, (int)(line), (msg))

// --- Basic assertions ---
#define TEST_ASSERT(c) \
    do { g_testsRun++; if(!(c)){UNITY_TEST_FAIL(__LINE__,"TEST_ASSERT failed: " #c);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_TRUE(c) \
    do { g_testsRun++; if(!(c)){UNITY_TEST_FAIL(__LINE__,"expected TRUE: " #c);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_FALSE(c) \
    do { g_testsRun++; if((c)){UNITY_TEST_FAIL(__LINE__,"expected FALSE: " #c);g_testsFailed++;}else{g_testsPassed++;}} while(0)

// --- Equality assertions ---
#define TEST_ASSERT_EQUAL(e,a) \
    do { g_testsRun++; if((e)!=(a)){UNITY_TEST_FAIL(__LINE__,"expected equal: " #e " != " #a);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_EQUAL_UINT8(e,a) \
    do { g_testsRun++; if((uint8_t)(e)!=(uint8_t)(a)){UNITY_TEST_FAIL(__LINE__,"UINT8 mismatch " #e "!=" #a);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_EQUAL_UINT16(e,a) \
    do { g_testsRun++; if((uint16_t)(e)!=(uint16_t)(a)){UNITY_TEST_FAIL(__LINE__,"UINT16 mismatch " #e "!=" #a);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_EQUAL_UINT32(e,a) \
    do { g_testsRun++; if((uint32_t)(e)!=(uint32_t)(a)){UNITY_TEST_FAIL(__LINE__,"UINT32 mismatch " #e "!=" #a);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_EQUAL_INT(e,a) \
    do { g_testsRun++; if((int)(e)!=(int)(a)){UNITY_TEST_FAIL(__LINE__,"INT mismatch " #e "!=" #a);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_EQUAL_INT32(e,a) \
    do { g_testsRun++; if((int32_t)(e)!=(int32_t)(a)){UNITY_TEST_FAIL(__LINE__,"INT32 mismatch " #e "!=" #a);g_testsFailed++;}else{g_testsPassed++;}} while(0)

// --- String assertions ---
#define TEST_ASSERT_EQUAL_STRING(e,a) \
    do { g_testsRun++; if(strcmp((const char*)(e),(const char*)(a))!=0){UNITY_TEST_FAIL(__LINE__,"STRING mismatch " #e "!=" #a);g_testsFailed++;}else{g_testsPassed++;}} while(0)

// --- Pointer assertions ---
#define TEST_ASSERT_NULL(p) \
    do { g_testsRun++; if((p)!=NULL){UNITY_TEST_FAIL(__LINE__,"expected NULL: " #p);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_NOT_NULL(p) \
    do { g_testsRun++; if((p)==NULL){UNITY_TEST_FAIL(__LINE__,"expected non-NULL: " #p);g_testsFailed++;}else{g_testsPassed++;}} while(0)

// --- Float assertions ---
#define TEST_ASSERT_EQUAL_FLOAT(e,a) \
    do { g_testsRun++; if((e)!=(a)){UNITY_TEST_FAIL(__LINE__,"FLOAT mismatch " #e "!=" #a);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_FLOAT_WITHIN(d,e,a) \
    do { float _udf=(float)(a)-(float)(e);if(_udf<0.0f)_udf=-_udf;if(_udf>(float)(d)){UNITY_TEST_FAIL(__LINE__,"FLOAT within " #d " failed");g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_EQUAL_DOUBLE(e,a) \
    do { g_testsRun++; if((double)(e)!=(double)(a)){UNITY_TEST_FAIL(__LINE__,"DOUBLE mismatch " #e "!=" #a);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_FLOAT_IS_INF(v) \
    do { g_testsRun++; if(!isinf((v))){UNITY_TEST_FAIL(__LINE__,"expected +INF: " #v);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_FLOAT_IS_NAN(v) \
    do { g_testsRun++; if(!isnan((v))){UNITY_TEST_FAIL(__LINE__,"expected NaN: " #v);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_FLOAT_NOT_INF(v) \
    do { g_testsRun++; if(isinf((v))){UNITY_TEST_FAIL(__LINE__,"expected NOT +INF: " #v);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_FLOAT_NOT_NAN(v) \
    do { g_testsRun++; if(isnan((v))){UNITY_TEST_FAIL(__LINE__,"expected NOT NaN: " #v);g_testsFailed++;}else{g_testsPassed++;}} while(0)

// --- Less than / Greater than ---
#define TEST_ASSERT_LESS_THAN(u,l) \
    do { g_testsRun++; if((uint32_t)(u)<=(uint32_t)(l)){UNITY_TEST_FAIL(__LINE__,"expected " #l "<" #u);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_GREATER_THAN(l,u) \
    do { g_testsRun++; if((uint32_t)(l)<=(uint32_t)(u)){UNITY_TEST_FAIL(__LINE__,"expected " #l ">" #u);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_GREATER_OR_EQUAL(l,u) \$
#define TEST_ASSERT_GREATER_OR_EQUAL(l,u) \
    do { g_testsRun++; if((uint32_t)(l)<(uint32_t)(u)){UNITY_TEST_FAIL(__LINE__,"expected " #l ">=" #u);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_NOT_EQUAL(e,a) \
    do { g_testsRun++; if((e)==(a)){UNITY_TEST_FAIL(__LINE__,"expected not equal");g_testsFailed++;}else{g_testsPassed++;}} while(0)

// --- Memory assertions ---
#define TEST_ASSERT_EQUAL_MEMORY(e,a,len) \
    do { g_testsRun++; if(memcmp((const void*)(e),(const void*)(a),(len))!=0){UNITY_TEST_FAIL(__LINE__,"MEMORY mismatch");g_testsFailed++;}else{g_testsPassed++;}} while(0)

// --- String contain assertions ---
#define TEST_ASSERT_INCLUDE(h,n) \
    do { g_testsRun++; if(strstr((const char*)(h),(const char*)(n))==NULL){UNITY_TEST_FAIL(__LINE__,"expected substring: " #n);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_NOT_INCLUDE(h,n) \
    do { g_testsRun++; if(strstr((const char*)(h),(const char*)(n))!=NULL){UNITY_TEST_FAIL(__LINE__,"expected NOT substring: " #n);g_testsFailed++;}else{g_testsPassed++;}} while(0)

// --- Message-suffixed assertions ---
#define TEST_ASSERT_EQUAL_MESSAGE(e,a,msg) \
    do { g_testsRun++; if((e)!=(a)){UNITY_TEST_FAIL(__LINE__,("FAIL: " msg));g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_TRUE_MESSAGE(c,msg) \
    do { g_testsRun++; if(!(c)){UNITY_TEST_FAIL(__LINE__,("FAIL: " msg));g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_FALSE_MESSAGE(c,msg) \
    do { g_testsRun++; if((c)){UNITY_TEST_FAIL(__LINE__,("FAIL: " msg));g_testsFailed++;}else{g_testsPassed++;}} while(0)

// --- Array assertions (compile-time count) ---
#define TEST_ASSERT_EQUAL_UINT8_ARRAY(e,a,cnt) \
    do { int _uuia; for(_uuia=0;_uuia<(cnt);_uuia++){if((e)[_uuia]!=(a)[_uuia]){UNITY_TEST_FAIL(__LINE__,"UINT8 ARRAY mismatch");g_testsFailed++;return;} g_testsPassed++;}} while(0)
#define TEST_ASSERT_EQUAL_UINT16_ARRAY(e,a,cnt) \
    do { int _uuib; for(_uuib=0;_uuib<(cnt);_uuib++){if((e)[_uuib]!=(a)[_uuib]){UNITY_TEST_FAIL(__LINE__,"UINT16 ARRAY mismatch");g_testsFailed++;return;} g_testsPassed++;}} while(0)
#define TEST_ASSERT_EQUAL_FLOAT_ARRAY(e,a,cnt) \
    do { int _uuc; for(_uuc=0;_uuc<(cnt);_uuc++){if((e)[_uuc]!=(a)[_uuc]){UNITY_TEST_FAIL(__LINE__,"FLOAT ARRAY mismatch");g_testsFailed++;return;} g_testsPassed++;}} while(0)
#define TEST_ASSERT_EQUAL_PTR_ARRAY(e,a,cnt) \
    do { int _uud; for(_uud=0;_uud<(cnt);_uud++){if((void*)(e)[_uud]!=(void*)(a)[_uud]){UNITY_TEST_FAIL(__LINE__,"PTR ARRAY mismatch");g_testsFailed++;return;} g_testsPassed++;}} while(0)

// --- Bit assertions ---
#define TEST_ASSERT_BITS(mask,e,a) \
    do { g_testsRun++; if(((mask)&(uint32_t)(e))!=((mask)&(uint32_t)(a))){UNITY_TEST_FAIL(__LINE__,"BITS mismatch");g_testsFailed++;}else{g_testsPassed++;}} while(0)

// --- Callback count ---
#define TEST_ASSERT_CALLBACK_COUNT(ec) \
    do { g_testsRun++; if(1!=(int)(ec)){UNITY_TEST_FAIL(__LINE__,"callback count mismatch");g_testsFailed++;}else{g_testsPassed++;}} while(0)

// --- Print test summary at end ---
static inline void unityPrintSummary(void) { \
  fprintf(stderr, "\n=== TEST SUMMARY ===\n"); \
  fprintf(stderr, "Tests run: %d\n", g_testsRun); \
  fprintf(stderr, "Passed: %d\n", g_testsPassed); \
  fprintf(stderr, "Failed: %d\n", g_testsFailed); \
  if(g_testsFailed==0) fprintf(stderr, "RESULT: ALL TESTS PASSED\n"); \
  else fprintf(stderr, "RESULT: %d FAILED\n", g_testsFailed); \
  fprintf(stderr, "===================\n\n"); \
}


// --- INT16 and INT32 aliases (for consistency) ---
#define TEST_ASSERT_EQUAL_INT16(e,a) \
    do { g_testsRun++; if((int16_t)(e)!=(int16_t)(a)){UNITY_TEST_FAIL(__LINE__,"INT16 mismatch " #e "!=" #a);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_EQUAL_INT32(e,a) \
    do { g_testsRun++; if((int32_t)(e)!=(int32_t)(a)){UNITY_TEST_FAIL(__LINE__,"INT32 mismatch " #e "!=" #a);g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_EQUAL_INT16_ARRAY(e,a,cnt) \
    do { int _uuia2; for(_uuia2=0;_uuia2<(cnt);_uuia2++){if((e)[_uuia2]!=(a)[_uuia2]){UNITY_TEST_FAIL(__LINE__,"INT16 ARRAY mismatch");g_testsFailed++;return;} g_testsPassed++;}} while(0)


#define TEST_ASSERT_LESS_THAN_UINT32(u,l) do { g_testsRun++; if((uint32_t)(u)<=(uint32_t)(l)){UNITY_TEST_FAIL(__LINE__,"UINT32");g_testsFailed++;}else{g_testsPassed++;}} while(0)
#define TEST_ASSERT_GREATER_THAN_UINT32(l,u) do { g_testsRun++; if((uint32_t)(l)<=(uint32_t)(u)){UNITY_TEST_FAIL(__LINE__,"UINT32");g_testsFailed++;}else{g_testsPassed++;}} while(0)
#endif /* UNITY_H */
