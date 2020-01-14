/*Senior Design Fall 2019
 * Athletic Performance Monitor Team*/
//__________________________
//Hardware Cnonfiguration   |
                            |
//MSP432        MAX30102    |
//P1.7   --->   SCL         |
//P1.6   --->   SDA         |
//p1.5   --->   INT         |
//3.3V   --->   VIN         |
//GND    --->   GND         |
//__________________________|

/* DriverLib Defines */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Defines */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>

/* Slave Address for I2C Slave */
#define MAX_ADDRESS       0x57

//Register Addresses
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONFIG 0x08
#define REG_MODE_CONFIG 0x09
#define REG_SPO2_CONFIG 0x0A
#define REG_LED1_PA 0x0C
#define REG_LED2_PA 0x0D
#define REG_PILOT_PA 0x10
#define REG_MULTI_LED_CTRL1 0x11
#define REG_MULTI_LED_CTRL2 0x12
#define REG_TEMP_INTR 0x1F
#define REG_TEMP_FRAC 0x20
#define REG_TEMP_CONFIG 0x21
#define REG_PROX_INT_THRESH 0x30
#define REG_REV_ID 0xFE
#define REG_PART_ID 0xFF

//function prototypes
void init_I2C();
void Reg_Write (uint8_t reg_address, uint8_t data);
void Reg_Read(uint8_t Reg_address);
void init_MAX();
void fifo_read();
void putsUART(char *s);
void config_interrupt();

//Other Constants
#define FIFO_DEPTH 96

/*Global Variables*/
uint32_t fifo_data_r[FIFO_DEPTH];//fifoData for red led buffer[number_of_samples]
uint32_t fifo_data_ir[FIFO_DEPTH];//fifoData for infra red led buffer[number_of_samples]
uint8_t reg_data;
char str[80];
uint8_t fifo_read_count = 0;
uint8_t reg_num; //1 to read one byte of data and 3 to read 3 bytes of data
uint8_t ready = 0;


/* I2C Master Configuration Parameter */
const eUSCI_I2C_MasterConfig i2cConfig =
{
        EUSCI_B_I2C_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        12000000,                                // SMCLK = 3MHz or 12MHz
        EUSCI_B_I2C_SET_DATA_RATE_400KBPS,      // Desired I2C Clock of 400khz or 100khz
        0,                                      // No byte counter threshold
        EUSCI_B_I2C_NO_AUTO_STOP                // No Autostop
};

/*UART COnfiguration Parameter for 9600 baud rate*/
const eUSCI_UART_ConfigV1 uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                     // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION,  // Oversampling
        EUSCI_A_UART_8_BIT_LEN                  // 8 bit data length
};

/*********************************************************************************************************/
/*Init I2C
 * This function initializes i2c on the msp432 based on the setting in the 12cConfig structure                                                                                         */
/*********************************************************************************************************/

void init_I2C(void)
{
    /* Select Port 1 for I2C - Set Pin 6, 7 to input Primary Module Function,
         *   (UCB0SIMO/UCB0SDA, UCB0SOMI/UCB0SCL).
         */
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                GPIO_PIN6 + GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);


        /* Initializing I2C Master to SMCLK at 400khz with no autostop */
        MAP_I2C_initMaster(EUSCI_B0_BASE, &i2cConfig);
        /* Specify slave address */
        MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, MAX_ADDRESS);

        /* Enable I2C Module to start operations */
        MAP_I2C_enableModule(EUSCI_B0_BASE);
       MAP_Interrupt_enableInterrupt(INT_EUSCIB0);
}
/*********************************************************************************************************/
/*config_interrupt  This function configures the interrupt pin on the MSP432                                                                                              */
/*********************************************************************************************************/
void config_interrupt()
{
    // Configuring P1.0 as output and P1.1 (switch) as input
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

        // Configuring P5.7 as an input and enabling interrupts
        MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P5, GPIO_PIN7);
        MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, GPIO_PIN7);
        MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P5,GPIO_PIN7,GPIO_HIGH_TO_LOW_TRANSITION);
        MAP_GPIO_enableInterrupt(GPIO_PORT_P5, GPIO_PIN7);
        MAP_Interrupt_enableInterrupt(INT_PORT5);
        // Enabling MASTER interrupts
        MAP_Interrupt_enableMaster();
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
}
void config_interrupttest()
{
    // Configuring P1.0 as output and P1.1 (switch) as input
        MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);

        // Configuring P5.7 as an input and enabling interrupts
        MAP_GPIO_setAsInputPinWithPullUpResistor(GPIO_PORT_P1, GPIO_PIN1);
        MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, GPIO_PIN1);
        //MAP_GPIO_interruptEdgeSelect(GPIO_PORT_P5,GPIO_PIN7,GPIO_HIGH_TO_LOW_TRANSITION);
        MAP_GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
        MAP_Interrupt_enableInterrupt(INT_PORT1);
        // Enabling MASTER interrupts
        MAP_Interrupt_enableMaster();
        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
}
/*********************************************************************************************************/
/*interrupt service routine  This function services the interrupt pin on the MSP432                                                                                              */
/*********************************************************************************************************/

void PORT5_IRQHandler(void)
{
    uint32_t status;
   // uint8_t i;

    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P5);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P5, status);

    // Toggling the output on the LED and calling the fifo_read function if the interrupt is reached
    if(status & GPIO_PIN7)
    {
         MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        fifo_read();
    }

}
void PORT1_IRQHandler(void)
{
    uint32_t status;


    status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    // Toggling the output on the LED and calling the fifo_read function if the interrupt is reached
    if(status & GPIO_PIN1)
    {
         MAP_GPIO_toggleOutputOnPin(GPIO_PORT_P1, GPIO_PIN0);
        fifo_read();

    }

}

/*********************************************************************************************************/
/* init_MAX   This function is used to initialize the MAX30102                                                                                  */
/*********************************************************************************************************/
void init_MAX()
{
    Reg_Write(REG_MODE_CONFIG,0x40); //Reset Device
    Reg_Read( REG_INTR_STATUS_1); //clear interrupt status
    Reg_Read( REG_INTR_STATUS_2);
    Reg_Write(REG_INTR_ENABLE_1,0xC0);//enable FIFO almost full and PPG ready
    Reg_Write(REG_INTR_ENABLE_2,0x00);
    Reg_Write(REG_FIFO_WR_PTR,0x00); //reset fifo write pointer
    Reg_Write(REG_OVF_COUNTER,0x00); //reset fifo overflow counter
    Reg_Write(REG_FIFO_RD_PTR,0x00); //reset fifo read pointer
    Reg_Write(REG_FIFO_CONFIG,0x40); //sample avg = 4, no fifo rollover, fifo almost full = 0
    Reg_Write(REG_SPO2_CONFIG,0x47);    //LSB = 31.25pA, sample rate = 100Hz, Pulse width 411us//was 0x27
    Reg_Write(REG_LED1_PA,0x5A); //LED current 7.8mA 0x24 or 0x27
    Reg_Write(REG_LED2_PA,0x5A); //LED current 7.2mA //not important for heart rate mode
    Reg_Write(REG_PILOT_PA,0x19);
    Reg_Write(REG_PROX_INT_THRESH,0x21);
    Reg_Write(REG_MODE_CONFIG,0x02); //heart rate mode //0x02 hr 0x03 spo2

}

/*********************************************************************************************************/
/* Reg_Write   This function is used to write to the registers in the MAX30102                                                                                  */
/*********************************************************************************************************/
void Reg_Write (uint8_t reg_address, uint8_t data)
{
      // MAP_I2C_setSlaveAddress(EUSCI_B0_BASE, HRM_WRITE_ADDRESS);
    /* Making sure the last transaction has been completely sent out */
       while (MAP_I2C_masterIsStopSent(EUSCI_B0_BASE));
       MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, reg_address);  // Start + 1Byte
       MAP_I2C_masterSendMultiByteFinish(EUSCI_B0_BASE,data);
       /*---------------------------------------------*/
}

/**********************************************************************************************************/
/* Reg_Read    This function is used to read from the registers in the MAX30102                                                                               */
/**********************************************************************************************************/
void Reg_Read(uint8_t Reg_address)
{
    reg_data = 0x00;
    reg_num = 1;

    MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, Reg_address);
    MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);
    MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
}


/*******************************************************************************
 * eUSCIB0 ISR. The repeated start and transmit/receive operations happen
 * within this ISR.
 *******************************************************************************/
void EUSCIB0_IRQHandler(void)
{
    uint_fast16_t status;
    uint8_t count;

    status = MAP_I2C_getEnabledInterruptStatus(EUSCI_B0_BASE);

    if (status & EUSCI_B_I2C_RECEIVE_INTERRUPT0)
        {
            if(reg_num == 1){//reg_num = 1 means we are just reading once from the specified register
                //reg_data = MAP_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
                MAP_I2C_disableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
                MAP_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
                reg_data = MAP_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);


            }
            else if (reg_num == 3) //this for loop stores a sample as 3 bytes of data
            {
                for(count = 0; count < FIFO_DEPTH - 1; count++)
                {
                    fifo_data_r[count] = 0;
                    fifo_data_ir[count] = 0;
                    fifo_data_r[count] = MAP_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE) & 3; //to clear unused bits

                    fifo_data_r[count] = (fifo_data_r[count] << 8) |MAP_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
                    fifo_data_r[count] = (fifo_data_r[count] << 8) |MAP_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
                }

                fifo_data_r[count] = 0;
                fifo_data_ir[count] = 0;
                fifo_data_r[count] = MAP_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE) & 3;
                fifo_data_r[count] = (fifo_data_r[count] << 8) |MAP_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);
                MAP_I2C_disableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
                MAP_I2C_masterReceiveMultiByteStop(EUSCI_B0_BASE);
                fifo_data_r[count] = (fifo_data_r[count] << 8) |MAP_I2C_masterReceiveMultiByteNext(EUSCI_B0_BASE);

                ready = 1; //When ready is = 1 the heart rate will be calculated and displayed to the UART
            }

        }

}

/**********************************************************************************************************/
/* fifo_read    This function is used to read from the registers in the MAX30102 and store in the fifo_data array                                                                             */
/**********************************************************************************************************/
void fifo_read()
{
    reg_num = 3;

    MAP_I2C_masterSendMultiByteStart(EUSCI_B0_BASE, REG_FIFO_DATA);
    MAP_I2C_masterReceiveStart(EUSCI_B0_BASE);
    MAP_I2C_enableInterrupt(EUSCI_B0_BASE, EUSCI_B_I2C_RECEIVE_INTERRUPT0);
}

/**********************************************************************************************************/
/* putsUART    This function is used to display a string to the UART                                                                              */
/**********************************************************************************************************/
void putsUART(char *s)
{
    while(*s)
        MAP_UART_transmitData(EUSCI_A0_BASE,*s++);
}
/**********************************************************************************************************/
/* init_UART    This function is used to initialize to the UART                                                                              */
/**********************************************************************************************************/
void init_UART()
{
    // Selecting P1.2 and P1.3 in UART mode
        MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                    GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

        // Setting DCO to 12MHz
        CS_setDCOCenteredFrequency(CS_DCO_FREQUENCY_12);

        // Configuring UART Module
         MAP_UART_initModule(EUSCI_A0_BASE, &uartConfig);

         // Enable UART module
         MAP_UART_enableModule(EUSCI_A0_BASE);
}


int main(void)
{
    /* Stop Watchdog  */
    MAP_WDT_A_holdTimer();
    uint8_t i;
    init_I2C();
    config_interrupt();
    config_interrupttest();
    init_UART();
    init_MAX();
    int peak_diff = 0;
    uint16_t peak_index = 0;
    float avg_peak_diff = 0;
    int sum = 0;
    uint16_t peak_num = 0;
    uint16_t heart_rate = 0;
    int mean = 0;
    int mean_Sum = 0;
    bool cond1;
    bool cond2;
    bool cond3;

     putsUART("\r\n               Senior Design HRM \0");
     while(1)
     {
         if(ready == 1){ //ready becomes 1 after the FIFO has been read three times (see line 316)
             ready = 0;
             mean = 0;
             mean_Sum = 0;
             peak_num = 0;
             for(i = 0; i < FIFO_DEPTH; i++)
             {
                 mean_Sum = mean_Sum + fifo_data_r[i];
             }
             mean = mean_Sum/FIFO_DEPTH;

             for(i = 1; i < FIFO_DEPTH - 3; i++)//peak detection
             {
                 cond1 = fifo_data_r[i-1] < fifo_data_r[i] && fifo_data_r[i+1] < fifo_data_r[i]; //sharp peak
                 cond2 = fifo_data_r[i-1] < fifo_data_r[i] && fifo_data_r[i+1] > fifo_data_r[i+2]; //flat peak
                 cond3 = fifo_data_r[i] > mean; //Ignore peaks below the Average for DC cancellation

                 if((cond1 || cond2) && cond3)  //if the value at index i is a peak
                    {
                         peak_num ++;
                         peak_diff = i - peak_index;
                         peak_index = i;
                         if(peak_num > 1)
                         {
                             sum = sum + peak_diff;
                             avg_peak_diff = sum/peak_num;
                         }
                    }
             }
             heart_rate = avg_peak_diff * 2.4; //2.4 = 60 x 0.04. the time between each sample is 0.04s
                for (i = 0; i < FIFO_DEPTH; i++)    //display ADC data on the uart terminal
                 {
                     sprintf(str, "\r\n %d \0",fifo_data_r[i]);
                     putsUART(str);
                 }

                 sprintf(str, "\r\nHeart Rate is %d bpm\0",heart_rate); //display heart rate on terminal
                 putsUART(str);
                 putsUART("\r\n\r\n\r\n");
         }
         else;

     }

}
