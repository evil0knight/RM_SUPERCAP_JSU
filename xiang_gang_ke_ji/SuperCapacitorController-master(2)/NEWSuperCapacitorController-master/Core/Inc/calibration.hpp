#pragma once

#ifndef HARDWARE_ID
#error "please specify hardware id"
#endif

//#define CALIBRATION_MODE
//#define IGNORE_CAPACITOR_ERROR

// [新增] IA/IB 动态零点变量前向声明，供 BIAS 宏引用
namespace PowerManager { namespace SampleManager { extern float iABiasDynamic; extern float iBBiasDynamic; } }

#if (HARDWARE_ID == 100)

// template for setting calibration values

#define VA_GAIN 1
#define VA_BIAS 0

#define VB_GAIN 1
#define VB_BIAS 0

#define I_REFEREE_GAIN 1
#define I_REFEREE_BIAS 0

#define I_A_CONVERT_GAIN 1
#define I_A_CONVERT_BIAS 0

#define I_B_CONVERT_GAIN 1
#define I_B_CONVERT_BIAS 0

// I_REFEREE should have (RefereeSystem -> DCDC_ASide) as positive
// I_ASide should have (DCDC_ASide -> DCDC_BSide) as positive
// I_BSide should have (DCDC_ASide -> DCDC_BSide) as positive

#elif (HARDWARE_ID == 101)

// todo: set calibration values

#define VA_GAIN 0.00760806f
#define VA_BIAS 0.257898f

#define VB_GAIN 0.00746000f
#define VB_BIAS 0.0184907f

#define I_REFEREE_GAIN 0.0194010f
#define I_REFEREE_BIAS (-39.7373f)

#define I_A_CONVERT_GAIN (0.019642f)
#define I_A_CONVERT_BIAS (PowerManager::SampleManager::iABiasDynamic)  // [新增] 动态零点，上电校准写入

#define I_B_CONVERT_GAIN (-0.0199702f)
#define I_B_CONVERT_BIAS (PowerManager::SampleManager::iBBiasDynamic)  // [新增] 动态零点，上电校准写入


// I_REFEREE should have (REFEREESystem -> DCDC_ASide) as positive
// I_Chassis should have (Chassis -> DCDC_ASide) as positive

#elif (HARDWARE_ID == 102)

// todo: set calibration values

#define VA_GAIN 0.0077488f
#define VA_BIAS 0.3460f

#define VB_GAIN 0.00805f
#define VB_BIAS (-0.09310f)

#define I_REFEREE_GAIN 0.020152f
#define I_REFEREE_BIAS (-41.13f)

#define I_A_CONVERT_GAIN (0.025916f)
#define I_A_CONVERT_BIAS (-48.593f)

#define I_B_CONVERT_GAIN (-0.0199702f)
#define I_B_CONVERT_BIAS 40.8592f

#elif (HARDWARE_ID == 201)

#define VA_GAIN 0.008062755f
#define VA_BIAS 0.007458192f

#define VB_GAIN 0.008073042f
#define VB_BIAS 0.024449784f

#define I_REFEREE_GAIN (0.020289855f)
#define I_REFEREE_BIAS (-41.48637681f)

#define I_A_CONVERT_GAIN (0.020068807f)
#define I_A_CONVERT_BIAS (-41.05332569f)

#define I_B_CONVERT_GAIN (-0.020080321f)
#define I_B_CONVERT_BIAS 41.05220884f

#elif (HARDWARE_ID == 202)

#define VA_GAIN 0.008080808f
#define VA_BIAS 0.016161616f

#define VB_GAIN 0.008106543f
#define VB_BIAS 0.034742328f

#define I_REFEREE_GAIN 0.020260492f
#define I_REFEREE_BIAS (-41.42981187f)

#define I_A_CONVERT_GAIN 0.020068807f
#define I_A_CONVERT_BIAS (-41.05332569f)

#define I_B_CONVERT_GAIN (-0.020097617f)
#define I_B_CONVERT_BIAS 41.09072639f

#elif (HARDWARE_ID == 203)

#define VA_GAIN 0.008078632f
#define VA_BIAS 0.018580854f

#define VB_GAIN 0.008072654f
#define VB_BIAS 0.039556004f

#define I_REFEREE_GAIN 0.020091848f
#define I_REFEREE_BIAS (-41.05912744f)

#define I_A_CONVERT_GAIN 0.020068807f
#define I_A_CONVERT_BIAS (-41.01920872f)

#define I_B_CONVERT_GAIN (-0.020063055f)
#define I_B_CONVERT_BIAS 41.00171969f

#elif (HARDWARE_ID == 204)

#define VA_GAIN 0.008101539f
#define VA_BIAS 0.025924926f

#define VB_GAIN 0.008090927f
#define VB_BIAS 0.035137738f

#define I_REFEREE_GAIN 0.020242915f
#define I_REFEREE_BIAS (-41.37044534f)

#define I_A_CONVERT_GAIN 0.02013809f
#define I_A_CONVERT_BIAS (-41.16340621f)

#define I_B_CONVERT_GAIN (-0.020178726f)
#define I_B_CONVERT_BIAS 41.25454021f

#elif (HARDWARE_ID == 205)


#define VA_GAIN 0.00807537f
#define VA_BIAS 0.02422611f

#define VB_GAIN 0.008096386f
#define VB_BIAS 0.041638554f

#define I_REFEREE_GAIN 0.020207852f
#define I_REFEREE_BIAS (-41.30051963f)

#define I_A_CONVERT_GAIN 0.020190366f
#define I_A_CONVERT_BIAS (-41.28785694f)

#define I_B_CONVERT_GAIN (-0.020219526f)
#define I_B_CONVERT_BIAS 41.32986713f

#endif

#define ADC_BUFFER_SIZE (ADC_CHANNEL_COUNT * ADC_SAMPLE_COUNT)

#define CALI_VA(vA_raw) (VA_GAIN * DIVIDED_BY_ADC_SAMPLE_COUNT * (vA_raw) + VA_BIAS)
#define CALI_VB(vB_raw) (VB_GAIN * DIVIDED_BY_ADC_SAMPLE_COUNT * (vB_raw) + VB_BIAS)
#define CALI_I_REFEREE(iReferee_raw) (I_REFEREE_GAIN * DIVIDED_BY_ADC_SAMPLE_COUNT * (iReferee_raw) + I_REFEREE_BIAS)
#define CALI_I_A_CONVERT(iAIn_raw) (I_A_CONVERT_GAIN * DIVIDED_BY_ADC_SAMPLE_COUNT * (iAIn_raw) + I_A_CONVERT_BIAS)
#define CALI_I_B_CONVERT(iCap_raw) (I_B_CONVERT_GAIN * DIVIDED_BY_ADC_SAMPLE_COUNT * (iCap_raw) + I_B_CONVERT_BIAS)
