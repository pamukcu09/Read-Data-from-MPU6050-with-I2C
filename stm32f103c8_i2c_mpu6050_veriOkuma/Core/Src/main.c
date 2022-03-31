// MPU6050 sensöründen I2C ile HAL kütüphaneleri kullanarak veri okuma.
// kaynak -> Youtube: Enes Çiçek
#include "main.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

//ADRES TANIMLAMALARI
#define MPU6050_ADDR 0x68<<1 // mpu6050 slave adresi 0x68. Fakat i2c de adresleme 7 bit olarak ayarlandı. Bu yüzden 1 sola kaydır.
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define GYRO_CNFG_REG 0x1B
#define ACC_CNFG_REG 0x1C
#define LPF_REG 0x1A


I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;

// DEĞİŞKEN TANIMLAMALARI
uint8_t data;
uint8_t buffer[2],tuffer[6],cuffer[6]; // ham gyro ve acc değerlerinin tutulduğu listeler
int16_t gyro_raw[3],acc_raw[3];        // anlamlandırılmış ham değerler
float gyro_cal[3],acc_cal[3]; // kalibrasyon offsetlerini tutan liste
int16_t raw_temp;
float temp;
int i;
float prevtime,prevtime1,time1,elapsedtime1,prevtime2,time2,elapsedtime2;
HAL_StatusTypeDef set_gyro;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);

int main(void)
{

  HAL_Init();


  SystemClock_Config();


  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();

  //********************************************************************************************************
  // POWER MANAGEMENT 1 REGISTER YAPILANDIRMASI PWR_MGMT_1
  data=0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &data, 1, HAL_MAX_DELAY);
  // 1) kullanılan birimin pointer olarak döndürülmesi,2) cihaz slave adresi, 3)register adresi,
  // 4) register boyutu, 5) verinin pointer olarak döndürülmesi,6) verinin boyutu , 7) zaman aşımı değeri
  //********************************************************************************************************

  //********************************************************************************************************
  //GYROSCOPE CONFIGURATION +-500 derece/saniye seçilecek. ilgili register'a 0x08 yazılmalı.(datasheetten ilgili bit yapılandırmaları)
  data=0x08;
  HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, GYRO_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);
  //********************************************************************************************************

  //********************************************************************************************************
  // ACCELEROMETER CONFIGURATION +-8g ölçüm aralığı seçilecek. ilgili registera 0x10 yazılmalı
  data=0x10;
  HAL_I2C_Mem_Write (&hi2c1, MPU6050_ADDR, ACC_CNFG_REG, 1, &data, 1, HAL_MAX_DELAY);

  for(i=0; i<2000; i++) // gyro sensörün durduğu zaman. herhangi bir şekilde dönmese bile döndürdüğü
     {					// değerler var.Bu döndürülen değerlerin ortalaması alınıp, asıl ölçüm değerinden
	  	  	  	  	  	// çıkarılmalı ve daha güvenilir bir sonuç elde edilmeli
   	  prevtime2 = time2;
   	  time2 = HAL_GetTick();
   	  elapsedtime2=(time2-prevtime2)*1000;

   	  cuffer[0]=0x43;
   	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,cuffer,1,HAL_MAX_DELAY); // Master STM32'den sensöre veri alacağının haberi veriliyor.
   	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,cuffer,6,HAL_MAX_DELAY); //0x43 adresinden itibaren 6 registerden veriler alınıyor

   	  gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]); // X ekseni
   	  gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]); // Y ekseni// Datasheet incelenerek, verilerin olması gerektiği bitlere ayarlandı
   	  gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]); // Z ekseni

   	  gyro_cal[0] += gyro_raw[0];
   	  gyro_cal[1] += gyro_raw[1]; // Döngüye göre 2000 veri toplanıyor.
   	  gyro_cal[2] += gyro_raw[2];

   	  HAL_Delay(3);

     }

     gyro_cal[0] /= 2000;
     gyro_cal[1] /= 2000;  // 2000 adet veri 2000'e bölünerek ortalamaları alınıyor
     gyro_cal[2] /= 2000;

     HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_13);
     HAL_Delay(1000);

  while (1)
  {
	  prevtime1 = time1;
	  time1 = HAL_GetTick();
	  elapsedtime1=(time1-prevtime1)*1000;

//********************************************************************************************************************

	  // ACCELEROMETER
	  tuffer[0]=0x3B; // Accelerometer ölçüm değerleri register başlangıç adresi
	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,tuffer,1,HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,tuffer,6,HAL_MAX_DELAY);

	  // Acc Raw Values
/*x ekseni */ acc_raw[0] = (tuffer[0] << 8 | tuffer[1]);
/*y ekseni */ acc_raw[1] = (tuffer[2] << 8 | tuffer[3]); // 8 bit veriler kaydırılarak ve OR işlemi yapılarak 16 bit veriler oluşturuluyor
/*z ekseni */ acc_raw[2] = (tuffer[4] << 8 | tuffer[5]);

//********************************************************************************************************************

	  // TEMPERATURE
	  buffer[0]=0x41;// Temperature ölçüm değeri register başlangıç adresi
	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,buffer,1,HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,buffer,2,HAL_MAX_DELAY);

	  // Temperature Values
	  raw_temp = (buffer[0] << 8 | buffer[1]);
	  temp = (raw_temp / 340.0) + 36.53; // datasheetteki hesaplama fonksiyonuna göre okunan veriden sıcaklık değeri elde ediliyor

//********************************************************************************************************************

	  // GYROSCOPE
	  cuffer[0]=0x43; // Gyroscope ölçüm değeri register başlangıç adresi
	  HAL_I2C_Master_Transmit(&hi2c1,MPU6050_ADDR,cuffer,1,HAL_MAX_DELAY);
	  HAL_I2C_Master_Receive(&hi2c1,MPU6050_ADDR,cuffer,6,HAL_MAX_DELAY);

	  	  // Gyro Raw Values
	  	  gyro_raw[0] = (cuffer[0] << 8 | cuffer[1]);
	  	  gyro_raw[1] = (cuffer[2] << 8 | cuffer[3]);
	  	  gyro_raw[2] = (cuffer[4] << 8 | cuffer[5]);

	  	  gyro_raw[0] -= gyro_cal[0];
	  	  gyro_raw[1] -= gyro_cal[1]; // ölçülen değerlerden, önceden hesaplanan kalibrasyon değerleri çıkarılıyor.
	  	  gyro_raw[2] -= gyro_cal[2];


	  	  /*
	  	   * Temperature verisi işlendi ve sıcaklık değeri elde edildi.
	  	   * Accelerometer ve Gyro verileri ise sadece okundu. İlgili değerler için matematiksel işlemlerden geçmeli
	  	   * 			STMSTUDIO ile veriler takip edilebilir.
	  	   */
	  	while((HAL_GetTick() - prevtime)*1000 < 4000);
	  	prevtime = HAL_GetTick();

  }

}


void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};


  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_I2C1_Init(void)
{


  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }


}


static void MX_USART1_UART_Init(void)
{


  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }


}


static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}


void Error_Handler(void)
{

  __disable_irq();
  while (1)
  {
  }

}

#ifdef  USE_FULL_ASSERT

void assert_failed(uint8_t *file, uint32_t line)
{

}
#endif


