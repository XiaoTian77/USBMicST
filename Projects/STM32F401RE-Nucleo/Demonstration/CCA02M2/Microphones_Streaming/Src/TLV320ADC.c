/*
 * TLV320ADC.c
 * TLV320ADC3100 control file
 *  Created on: Jan 22, 2021
 *      Author: Joaquin
 */

#define ARM_MATH_CM4

#include "TLV320ADC.h"
#include "trigTable.h"
#include "arm_math.h"




#define TEST_LENGTH_SAMPLES   96  //BLOCK_SIZE_float
#define BLOCK_SIZE            96  //BLOCK_SIZE_float
#define NUM_TAPS              17

uint32_t blockSize = BLOCK_SIZE;
uint32_t numBlocks = TEST_LENGTH_SAMPLES/BLOCK_SIZE;

float32_t testOutput[TEST_LENGTH_SAMPLES*2];
float32_t testInput[TEST_LENGTH_SAMPLES*2];
float32_t firStateF32[BLOCK_SIZE + NUM_TAPS - 1];


// 17 taps low pass, cutoff at about 8000Hz
static float firCoeffs32[NUM_TAPS] = {
		 -0.0021834891907904987,
		  0.023133081888390004,
		  0.03440125360693663,
		  0.054016706019288735,
		  0.07610902012650608,
		  0.09772535709704201,
		  0.11593264129629442,
		  0.12810228628568973,
		  0.13238343618749146,
		  0.12810228628568973,
		  0.11593264129629442,
		  0.09772535709704201,
		  0.07610902012650608,
		  0.054016706019288735,
		  0.03440125360693663,
		  0.023133081888390004,
		  -0.0021834891907904987
};

// 27 taps high pass
//static float firCoeffs32[NUM_TAPS] = {
//-0.023407821791635055,
//0.007483605671744372,
//0.012776270582352792,
//0.0191120618729217,
//0.023280321680242914,
//0.02185571261554037,
//0.012298820160004144,
//-0.00661626100726055,
//-0.034073792449169624,
//-0.0671219042059157,
//-0.10117559002347454,
//-0.13081889276741943,
//-0.1510059949963812,
//0.8418301719223038,
//-0.1510059949963812,
//-0.13081889276741943,
//-0.10117559002347454,
//-0.0671219042059157,
//-0.034073792449169624,
//-0.00661626100726055,
//0.012298820160004144,
//0.02185571261554037,
//0.023280321680242914,
//0.0191120618729217,
//0.012776270582352792,
//0.007483605671744372,
//-0.023407821791635055
//};


arm_fir_instance_f32 S;
float32_t  *inputF32, *outputF32;


extern  uint16_t PCM_Buffer[];
extern  uint16_t I2S_InternalBuffer[];
extern  HAL_StatusTypeDef AUDIO_IN_Timer_Init(void);


/**
  * @brief  TLV320ADC3100 init function
  * @note   You can check the TLV320ADC.h for more register fucntion
  * @retval Init status
  */

void ADC_init(){

	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,0);  //reset
	HAL_Delay(10);
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,1);  //reset

	I2C_Buffer_Read =0x00;

	I2C_Buffer_Write = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x00, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100); //page 0
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x00, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}

	I2C_Buffer_Write = R0x01_0;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x01, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100); //soft reset


	I2C_Buffer_Write = R0x12_0;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x12, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100); //NADC 1
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x12, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


	I2C_Buffer_Write = R0x13_0;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x13, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);	//MADC 2
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x13, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


	I2C_Buffer_Write = R0x14_0;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x14, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);	//AOSR 128
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x14, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


	I2C_Buffer_Write = R0x1B_0;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x1B, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);	//ADC Audio Interface Control
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x1B, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


//	I2C_Buffer_Write = R0x35_0;
//	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x35, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);	//DOUT bus keeper enabled, primary DOUT output for codec interface

	I2C_Buffer_Write = 0x01;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x00, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100); //page 1
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x00, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


	I2C_Buffer_Write = R0x33_1;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x33, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100); //MICBIAS 2.5V
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x33, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


	I2C_Buffer_Write = R0x3B_1;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x3B, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100); //Left PGA on and gain
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x3B, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


	I2C_Buffer_Write = R0x3C_1;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x3C, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);//Right PGA on and gain
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x3C, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


	I2C_Buffer_Write = R0x34_1;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x34, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);//Left PGA input selection
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x34, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


	I2C_Buffer_Write = R0x36_1;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x36, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);//Left PGA input selection 2
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x36, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


	I2C_Buffer_Write = R0x37_1;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x37, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);//Right PGA input selection
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x37, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


	I2C_Buffer_Write = R0x39_1;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x39, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);//Right PGA input selection 2
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x39, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}




}


void ADC_start(){


	I2C_Buffer_Write = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x00, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100); //page 0
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x00, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}



	I2C_Buffer_Write = R0x51_0;                                               //power on
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x51, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x51, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}



	I2C_Buffer_Write = R0x52_0;
	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x52, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);
	HAL_I2C_Mem_Read(&hi2c1, Write_device_address, 0x52, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 1, 100);
	if (I2C_Buffer_Read!=I2C_Buffer_Write){
		while(1);
	}


//	I2C_Buffer_Write = R0x53_0;
//	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x53, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);
//
//	I2C_Buffer_Write = R0x54_0;
//	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x54, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 1, 100);

//	I2C_Buffer_Write = 0x01;
//	HAL_I2C_Mem_Write(&hi2c1, Write_device_address, 0x00, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Write, 8, 100); //page 1
//	HAL_I2C_Mem_Read(&hi2c1, Read_device_address, 0x3B, I2C_MEMADD_SIZE_8BIT, &I2C_Buffer_Read, 8, 100);

}



void HAL_I2S_RxHalfCpltCallback(I2S_HandleTypeDef *hi2s){


	uint16_t * DataTempI2S = &I2S_InternalBuffer;
    memcpy(AudioInCtx[0].pBuff, DataTempI2S, 192);
	CCA02M2_AUDIO_IN_HalfTransfer_CallBack(0);
}


void HAL_I2S_RxCpltCallback(I2S_HandleTypeDef *hi2s){


	uint16_t * DataTempI2S = &(I2S_InternalBuffer[AudioInCtx[0].Size/2U]);
    memcpy(AudioInCtx[0].pBuff, DataTempI2S, 192);
	CCA02M2_AUDIO_IN_HalfTransfer_CallBack(0);


}

void fir_init(void){
	arm_fir_init_f32(&S, NUM_TAPS, &firCoeffs32[0], &firStateF32[0], blockSize);
}

void fir_process(int16_t* inbuff, int16_t* outbuff, size_t size)
{

	size_t i, index;

    for (index=0;index < size; index++)
    {
    	testInput[index] = (float32_t)(inbuff[index]);
    }
    inputF32 = &testInput[0];
 	outputF32 = &testOutput[0];
 	arm_fir_f32(&S, inputF32, outputF32, blockSize);
    for (i=0;i<size;i++)
    {
    	outbuff[i] = ((int16_t)testOutput[i]);
    }
}

