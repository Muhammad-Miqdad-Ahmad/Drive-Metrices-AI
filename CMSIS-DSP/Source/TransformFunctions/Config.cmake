cmake_minimum_required (VERSION 3.14)



target_sources(CMSISDSP PRIVATE TransformFunctions/arm_bitreversal.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_bitreversal2.c)

if ((NOT ARMAC5) AND (NOT DISABLEFLOAT16))
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_bitreversal_f16.c)
endif() 

if (F32)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix2_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix8_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_init_f32.c)
endif()

if ((NOT ARMAC5) AND (NOT DISABLEFLOAT16))
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix2_f16.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_f16.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_f16.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_init_f16.c)
endif()

if ((NOT ARMAC5) AND (NOT DISABLEFLOAT16))
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_init_f16.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_f16.c)
endif()

if (F64)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_f64.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_init_f64.c)
endif()

if (Q15)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix2_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_init_q15.c)
endif()

if (Q31)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix2_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_init_q31.c)
endif()

if (F32)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_dct4_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_dct4_init_f32.c)

target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_init_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_init_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_f32.c)
endif()

if (Q31)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_dct4_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_dct4_init_q31.c)

target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_init_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_init_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_init_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_q31.c)
endif()

if (Q15)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_dct4_init_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_dct4_q15.c)

target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_init_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_init_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_init_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_q15.c)
endif()

if (F32)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_fast_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_fast_init_f32.c)
if (CFFT)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_init_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix8_f32.c)
endif()
endif()

if (F64)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_fast_f64.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_fast_init_f64.c)
endif()

if ((NOT ARMAC5) AND (NOT DISABLEFLOAT16))
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_fast_f16.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_fast_init_f16.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_f16.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_init_f16.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix8_f16.c)
endif()

if (F32)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_init_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_init_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_f32.c)
endif()

if (Q15)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_init_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_init_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_q15.c)
endif()

if (Q31)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_init_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_rfft_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_init_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_q31.c)
endif()

if (WRAPPER)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix2_init_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix2_init_q31.c)
endif()

if (Q15)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_init_q15.c)
endif()
if (Q31)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix4_init_q31.c)
endif()


# For scipy or wrappers or benchmarks
if (WRAPPER)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix2_init_f32.c)
if ((NOT ARMAC5) AND (NOT DISABLEFLOAT16))
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_cfft_radix2_init_f16.c)
endif()

    
endif()

if (MFCC)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_mfcc_init_f32.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_mfcc_f32.c)

target_sources(CMSISDSP PRIVATE TransformFunctions/arm_mfcc_init_q31.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_mfcc_q31.c)

target_sources(CMSISDSP PRIVATE TransformFunctions/arm_mfcc_init_q15.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_mfcc_q15.c)

if ((NOT ARMAC5) AND (NOT DISABLEFLOAT16))
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_mfcc_init_f16.c)
target_sources(CMSISDSP PRIVATE TransformFunctions/arm_mfcc_f16.c)
endif()
endif()



