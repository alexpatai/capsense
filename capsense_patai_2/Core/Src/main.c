/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * Overview: this project is about using the Capsense CY8C201A0 click board, that
 * has 2 buttons and a slider. The buttons have hollow centers, where leds are
 * mounted, to indicate the status of the button. Furthermore, every button and
 * slider are capacitive touch sensors, that measure the capacitance of the human
 * body that comes in contact with them. If this capacitance exceeds a certain
 * threshold, it is interpreted as, the button or slider has been touched by
 * the user. The slider consists of 5 segments, which means, when touched, 5
 * different values can be measured by the slider.
 *
 * Functionality:
 * - the user is able to turn the buttons on by touching them. It
 * is possible to touch both at the same time, their behaviour does not change.
 * The buttons switch, when they are released, just like a smartphone touch screen.
 * I found this one, the best method of implementation, because this serves a
 * debouncing tool too (the microcontroller will not switch the button/led on
 * and off several times if the buttons are held longer).
 * - The slider consists of 5 segments, and their measurements all differ from
 * each other. Unfortunately, the way these values progress is not linear and
 * I couldn't find a way to re-calibrate it. So the values, measured by the
 * slider segments, from left to right, are the following:4->12->28->24->16.
 * I have assigned percentage values to these 5 values, just to symbolize, how
 * an OLED screen's brightness would be controlled by this slider.
 * 4 = 0% -> lowest brightness/top of the scrollable text
 * 12 = 25% -> 25% of the max brightness/scrolling down to 20%
 * 28 = 50% -> 50% of the max brightness/scrolling further down to 50%
 * 24 = 20% -> 75% of the max brightness/scrolling further down to 75%
 * 16 = 20% -> 100% of the max brightness/scrolling further down to the bottom of the text
 * That is how we would have controlled brightness of the OLED screen by the slider or
 * as in the project concept described, the way the text would have been scrolled on
 * the OLED screen.
 *
 * UART connection: Baud Rate = 115200
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAPSENSE_COMMAND_REG                      0xA0
#define CAPSENSE_CS_ENABL0                        0x06
#define CAPSENSE_CS_ENABL1                        0x07
#define CAPSENSE_GPIO_ENABLE0                     0x08	//bei initialisierung 0x03 -> 2 GPIO Pins: GP0[0] und GP0[1]
#define CAPSENSE_DM_STRONG0                       0x11
#define CAPSENSE_CS_SLID_CONFIG                   0x75
#define CAPSENSE_CS_SLID_MULM                     0x77
#define CAPSENSE_CS_SLID_MULL                     0x78
#define CAPSENSE_OUTPUT_PORT0                     0x04 //bei initialisierung 0x03
#define CAPSENSE_CS_READ_RAW                      0x87	//mit 0x86 ausprobieren
#define CAPSENSE_CS_READ_STATUS0                  0x88
#define CAPSENSE_CS_READ_STATUS1                  0x89
#define CAPSENSE_CS_READ_CEN_POSM                 0x8A
#define CAPSENSE_CS_READ_CEN_POSL                 0x8B
#define CAPSENSE_CS_READ_CEN_PEAKM                0x8C
#define CAPSENSE_CS_READ_CEN_PEAKL                0x8D
#define CAPSENSE_DEVICE_ID                        0x7A
#define CAPSENSE_DEVICE_STATUS                    0x7B

#define CAPSENSE_I2C_ADDRESS_0                    0x00		//
#define CAPSENSE_I2C_ADDRESS_1                    0x4B		//
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

//for reading the raw values of the buttons
uint8_t read[2] = { 0 };

//for reading the actual status of the buttons
uint8_t button_status;

//for reading the slider values
uint8_t read_slider[2] = { 0 };

//for a string representation of the slider values and a message
uint8_t slider_value[30];

//to see, if a led must be toggled. Set by the MasterReceive interrupt
uint8_t flag = 0;

//last value of the slider
uint8_t last_val = 137;	//137 is the initial value of the CS_READ_STATUS1 register, also read_slider[0]

//to see if the slider value has changed. Set by the MasterReceive interrupt
uint8_t flag_slider = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
/**
 * Initializes the click board with its basic settings, needed for the project.
 * 2 GPIO pins are enabled for the 2 LEDs in the buttons. These can be toggled
 * by toggling CAPSENSE_OUTPUT_PORT0. Furthermore the Capsense buttons and slider
 * are also enabled. All of this happens in setup mode. The config is saved and
 * then the mode of operation is changed to normal.
 * @param hi2c The I2C connection.
 */
HAL_StatusTypeDef init_capsense(I2C_HandleTypeDef *hi2c);

/**
 * The master writes to the slave.
 * @param hi2c The I2C connection.
 * @param reg Indicates which register to write to.
 * @param data_buf An array of unsigned integers of 8bit, that contain the value that is written to the register.
 * @param len The size of the data_buf array.
 */
HAL_StatusTypeDef Capsense_Write(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *data_buf, uint8_t len);

/**
 * The master writes to the slave and then the slave answers with either one or two bytes of data.
 * This function has two modes, when the parameter mode==1, the function triggers an interrupt, when
 * the mode==2, the function does not trigger any I2C interrupts.
 * @param hi2c The I2C connection.
 * @param reg Indicates which register to write to.
 * @param value An array of unsigned integers of 8bit, that contain the value that is read from the register.
 * @param len The size of the value array.
 * @param mode Decides if the function triggers an interrupt. (Interrupt: mode = 1)
 */
HAL_StatusTypeDef Capsense_Read(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *value, uint8_t len, uint8_t mode);

/**
 * Starts the scanning process. This must be done, so that the click modul "listens" to the buttons and sliders.
 * It basically starts the sensors of the click module.
 * @param hi2c The I2C connection.
 */
HAL_StatusTypeDef startScan(I2C_HandleTypeDef *hi2c);

/**
 * If the flag global variable has been set to 1 in the interrupt service routine, this function will toggle poll
 * the buttons once more, to see which one has been touched and that button's led will be then toggled. It is
 * possible to toggle both buttons and their leds at the same time.
 * This function polls the buttons as long as they are let go, just like smartphones do, so one touch event counts
 * as one event, however long that may take.
 * @param hi2c The I2C connection.
 */
uint8_t toggle_leds(I2C_HandleTypeDef *hi2c);

/**
 * Polls the buttons to get an overview, which button is in which state, and writes the acquired states to the
 * UART interface in a string/readable format. This function is only triggered if the state of at least one button
 * is changing.
 * @param hi2c The I2C connection.
 */
void get_button_status(I2C_HandleTypeDef *hi2c);

/**
 * Polls the slider, to get the value where the user's finger is. It converts the measured values to a string/readable
 * format and transmits it to the UART interface. This function runs as long as the user holds their finger on the
 * slider.
 * @param hi2c The I2C connection.
 */
void get_slider_status(I2C_HandleTypeDef *hi2c);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration--------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_I2C1_Init();
	MX_USART2_UART_Init();
	/* USER CODE BEGIN 2 */
	uint8_t status[30] = "UART INIT OK\r\n";
	uint8_t init_ok[] = "INIT OK\r\n";
	uint8_t init_nok[] = "INIT NOK\r\n";

	HAL_StatusTypeDef ret;
	//uint8_t led_changed;
	HAL_UART_Transmit(&huart2, status, strlen((char*) status), 1000);

	HAL_StatusTypeDef success = init_capsense(&hi2c1);
	if (success == HAL_OK) {
		HAL_UART_Transmit(&huart2, init_ok, strlen((char*) init_ok), 1000);
	} else {
		HAL_UART_Transmit(&huart2, init_nok, strlen((char*) init_nok), 1000);
	}

	HAL_Delay(500);

	ret = startScan(&hi2c1);
	if (ret == HAL_OK) {
		sprintf((char*) status, "Start scanning...\r\n\n");
	} else {
		sprintf((char*) status, "ScanFail\r\n");
	}
	HAL_UART_Transmit(&huart2, status, strlen((char*) status), 1000);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		ret = Capsense_Read(&hi2c1, CAPSENSE_CS_READ_RAW, read, 2, 1); //Reads the raw values of the Capsense register, and triggers an interrupt.
		HAL_Delay(10);
		toggle_leds(&hi2c1); //Checks the flag variable, to see if one or more leds need to be toggled.
		Capsense_Read(&hi2c1, CAPSENSE_CS_READ_STATUS1, read_slider, 2, 1);
		HAL_Delay(10);
		get_slider_status(&hi2c1); //Checks the slider, if the user has a finger on it.

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure LSE Drive Capability
	 */
	HAL_PWR_EnableBkUpAccess();
	__HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE
			| RCC_OSCILLATORTYPE_MSI;
	RCC_OscInitStruct.LSEState = RCC_LSE_ON;
	RCC_OscInitStruct.MSIState = RCC_MSI_ON;
	RCC_OscInitStruct.MSICalibrationValue = 0;
	RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
	RCC_OscInitStruct.PLL.PLLM = 1;
	RCC_OscInitStruct.PLL.PLLN = 16;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}
	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		Error_Handler();
	}
	/** Enable MSI Auto calibration
	 */
	HAL_RCCEx_EnableMSIPLLMode();
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00707DBD;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_DISABLE)
			!= HAL_OK) {
		Error_Handler();
	}
	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */

	/* USER CODE END USART2_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LD3_Pin */
	GPIO_InitStruct.Pin = LD3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
 * Polls the slider, to get the value where the user's finger is. It converts the measured values to a string/readable
 * format and transmits it to the UART interface. This function runs as long as the user holds their finger on the
 * slider.
 * @param hi2c The I2C connection.
 */
void get_slider_status(I2C_HandleTypeDef *hi2c) { //Slider values: 4->12->28->24->16.

	if (hi2c == &hi2c1 && flag_slider==1 && read_slider[0]!=137) {
		Capsense_Read(hi2c, CAPSENSE_CS_READ_STATUS1, read_slider, 2, 2);
		while(read_slider[0] == last_val){	//eliminates duplicate measurements
			HAL_Delay(10);
			Capsense_Read(hi2c, CAPSENSE_CS_READ_STATUS1, read_slider, 2, 2);
		}
		if (read_slider[0] != 0) {
			uint8_t percentage;
			switch (read_slider[0]) {
			case 0:
				percentage = 0;
				break;
			case 4:
				percentage = 0;
				break;
			case 12:
				percentage = 25;
				break;
			case 28:
				percentage = 50;
				break;
			case 24:
				percentage = 75;
				break;
			case 16:
				percentage = 100;
				break;
			}
			sprintf((char*) slider_value, "Slider is at : %u%%\r\n",
					percentage);
			HAL_UART_Transmit(&huart2, slider_value,
					strlen((char*) slider_value),
					HAL_MAX_DELAY);
			last_val = read_slider[0];
			flag_slider=0;
		}
	}

}

/**
 * Polls the buttons to get an overview, which button is in which state, and writes the acquired states to the
 * UART interface in a string/readable format. This function is only triggered if the state of at least one button
 * is changing.
 * @param hi2c The I2C connection.
 */
void get_button_status(I2C_HandleTypeDef *hi2c) {

	if (hi2c == &hi2c1) {
		uint8_t stat = 0;
		uint8_t str[53];

		Capsense_Read(hi2c, 0x00, &stat, 1, 2);
		HAL_Delay(10);
		switch (stat) {
		case 0:
			sprintf((char*) str, "Both buttons/leds are off.\r\n\n");
			break;
		case 1:
			sprintf((char*) str,
					"Top button/led is on.\r\nBottom button/led is off.\r\n\n");
			break;
		case 2:
			sprintf((char*) str,
					"Top button/led is off.\r\nBottom button/led is on.\r\n\n");
			break;
		case 3:
			sprintf((char*) str, "Both buttons/leds are on.\r\n\n");
			break;
		}

		HAL_UART_Transmit(&huart2, str, strlen((char*) str), 1000);
	}

}

/**
 * If the flag global variable has been set to 1 in the interrupt service routine, this function will toggle poll
 * the buttons once more, to see which one has been touched and that button's led will be then toggled. It is
 * possible to toggle both buttons and their leds at the same time.
 * This function polls the buttons as long as they are let go, just like smartphones do, so one touch event counts
 * as one event, however long that may take.
 * @param hi2c The I2C connection.
 */
uint8_t toggle_leds(I2C_HandleTypeDef *hi2c) {

	if (hi2c == &hi2c1) {
		uint8_t return_value = 0;
		uint8_t button;

		Capsense_Read(hi2c, 0x00, &button_status, 1, 2);
		HAL_Delay(10);

		if (flag == 1) {
			uint8_t cmd = button_status;
			uint8_t button_hold = read[1];
			while (button_hold != 0x0 && button_hold != 0x1
					&& button_hold != 0x2 && button_hold != 0x3) { //"debounce". buttons react for letting go, just like smartphones
				Capsense_Read(hi2c, 0x00, &button_hold, 1, 2);
				HAL_Delay(10);
			}

			if (read[1] == 16) {
				cmd = button_status ^ 0x1;
				Capsense_Write(hi2c, CAPSENSE_OUTPUT_PORT0, &cmd, 1);
				HAL_Delay(10);
				return_value = 1;
			}
			if (read[1] == 8) {
				cmd = button_status ^ 0x2;
				Capsense_Write(hi2c, CAPSENSE_OUTPUT_PORT0, &cmd, 1);
				HAL_Delay(10);
				return_value = 1;
			}
			if (read[1] == 24) {
				Capsense_Read(hi2c, 0x00, &button, 1, 2);
				cmd = button ^ 0x01;
				Capsense_Write(hi2c, CAPSENSE_OUTPUT_PORT0, &cmd, 1);
				HAL_Delay(10);
				Capsense_Read(hi2c, 0x00, &button, 1, 2);
				cmd = button ^ 0x02;
				HAL_Delay(10);
				Capsense_Write(hi2c, CAPSENSE_OUTPUT_PORT0, &cmd, 1);
				return_value = 1;
			}
			flag = 0;
			HAL_Delay(250);
		}
		if (return_value == 1)
			get_button_status(hi2c);

		return return_value;
	} else
		return 99;
}

/**
 * Sets the flag to 1 if the slave has completely transmitted a message to the master.
 * This flag indicates to other functions, to check the buttons, of they have been pressed,
 * because the LEDs must be toggled then.
 * @param hi2c The I2C connection.
 */
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) { //flag setzen und toggle von led in main
	if (hi2c == &hi2c1) {
		flag = 1;
		if(read_slider[0] != last_val && read_slider[0]!=137)
			flag_slider = 1;
	}
}

/**
 * Initializes the click board with its basic settings, needed for the project.
 * 2 GPIO pins are enabled for the 2 LEDs in the buttons. These can be toggled
 * by toggling CAPSENSE_OUTPUT_PORT0. Furthermore the Capsense buttons and slider
 * are also enabled. All of this happens in setup mode. The config is saved and
 * then the mode of operation is changed to normal.
 * @param hi2c The I2C connection.
 */
HAL_StatusTypeDef init_capsense(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef ret;

	if (hi2c == &hi2c1) {
		uint8_t command = 0x08;
		ret = Capsense_Write(hi2c, CAPSENSE_COMMAND_REG, &command, 1);

		command = 0x1F;
		ret = Capsense_Write(hi2c, CAPSENSE_CS_ENABL1, &command, 1);

		command = 0x18;
		ret = Capsense_Write(hi2c, CAPSENSE_CS_ENABL0, &command, 1);

		command = 0x03;
		ret = Capsense_Write(hi2c, CAPSENSE_GPIO_ENABLE0, &command, 1);
		ret = Capsense_Write(hi2c, CAPSENSE_DM_STRONG0, &command, 1);

		command = 0x01;
		ret = Capsense_Write(hi2c, CAPSENSE_CS_SLID_CONFIG, &command, 1);

		command = 0x64;
		ret = Capsense_Write(hi2c, CAPSENSE_CS_SLID_MULM, &command, 1);

		command = 0x00;
		ret = Capsense_Write(hi2c, CAPSENSE_CS_SLID_MULL, &command, 1);

		command = 0x01;
		ret = Capsense_Write(hi2c, CAPSENSE_COMMAND_REG, &command, 1);

		HAL_Delay(100);
		command = 0x06;
		ret = Capsense_Write(hi2c, CAPSENSE_COMMAND_REG, &command, 1);

		HAL_Delay(100);
		command = 0x03;
		ret = Capsense_Write(hi2c, CAPSENSE_OUTPUT_PORT0, &command, 1);

		HAL_Delay(100);
		command = 0x07;
		ret = Capsense_Write(hi2c, CAPSENSE_OUTPUT_PORT0, &command, 1);

		HAL_Delay(100);
	} else {
		ret = !HAL_OK;
	}

	return ret;
}

/**
 * The master writes to the slave.
 * @param hi2c The I2C connection.
 * @param reg Indicates which register to write to.
 * @param data_buf An array of unsigned integers of 8bit, that contain the value that is written to the register.
 * @param len The size of the data_buf array.
 */
HAL_StatusTypeDef Capsense_Write(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *data_buf, uint8_t len) {
	if (hi2c == &hi2c1) {
		uint8_t tx_buffer[256];
		uint8_t cnt;

		tx_buffer[0] = reg;
		for (cnt = 1; cnt <= len; cnt++)
			tx_buffer[cnt] = data_buf[cnt - 1];

		//for(uint8_t i=0x00; i<=0xFF; i++){
		return HAL_I2C_Master_Transmit(hi2c, CAPSENSE_I2C_ADDRESS_0, tx_buffer,
				len + 1, HAL_MAX_DELAY);
	} else
		return !HAL_OK;
}

/**
 * The master writes to the slave and then the slave answers with either one or two bytes of data.
 * This function has two modes, when the parameter mode==1, the function triggers an interrupt, when
 * the mode==2, the function does not trigger any I2C interrupts.
 * @param hi2c The I2C connection.
 * @param reg Indicates which register to write to.
 * @param value An array of unsigned integers of 8bit, that contain the value that is read from the register.
 * @param len The size of the value array.
 * @param mode Decides if the function triggers an interrupt. (Interrupt: mode = 1)
 */
HAL_StatusTypeDef Capsense_Read(I2C_HandleTypeDef *hi2c, uint8_t reg,
		uint8_t *value, uint8_t len, uint8_t mode) {

	if (hi2c == &hi2c1) {
		HAL_StatusTypeDef ret;
		value[0] = reg;
		uint8_t status[16];

		//Write the register address first
		ret = HAL_I2C_Master_Transmit(hi2c, CAPSENSE_I2C_ADDRESS_0, value, 1,
		HAL_MAX_DELAY);

		if (ret != HAL_OK) {
			sprintf((char*) status, "ReadErr\r\n");
			HAL_UART_Transmit(&huart2, status, strlen((char*) status), 1000);
		} else {
			if (mode == 1)
				ret = HAL_I2C_Master_Receive_IT(hi2c,
				CAPSENSE_I2C_ADDRESS_0 | 0x01, value, len);
			else
				ret = HAL_I2C_Master_Receive(hi2c,
				CAPSENSE_I2C_ADDRESS_0 | 0x01, value, len,
				HAL_MAX_DELAY);
			if (ret != HAL_OK) {
				sprintf((char*) status, "ReadErr\r\n");
			} else {
				sprintf((char*) status, "%u - %u\r\n", (uint8_t) value[0],
						(uint8_t) value[1]);
			}

		}
		return ret;
	}
	return !HAL_OK;
}

/**
 * Starts the scanning process. This must be done, so that the click modul "listens" to the buttons and sliders.
 * It basically starts the sensors of the click module.
 * @param hi2c The I2C connection.
 */
HAL_StatusTypeDef startScan(I2C_HandleTypeDef *hi2c) {
	HAL_StatusTypeDef ret;
	uint8_t cmd = 0x09;

	ret = Capsense_Write(hi2c, CAPSENSE_COMMAND_REG, &cmd, 1);
	return ret;

}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
