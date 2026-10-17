#include "oled_hardware_i2c.h"
#include "oledfont.h"
#include "i2c.h"


void delay_ms(uint32_t ms)
{
    HAL_Delay(ms);
}

// ---------------------------------------------------------
// 【STM32 I2C 死锁解锁机制】
// 当遇到复位时 OLED 没掉电导致 SDA 被拉低死锁的情况，
// 通过将 PB6(SCL) 转为 GPIO 并发送最多 9 个时钟脉冲，让从机释放 SDA 总线
// ---------------------------------------------------------
void oled_i2c_sda_unlock(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. 先关闭复位 I2C 外设
    HAL_I2C_DeInit(&hi2c1);

    // 2. 将 PB7 (SDA) 和 PB6 (SCL) 配置为普通开漏输出模式
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(OLED_I2C_GPIO_PORT, OLED_I2C_SCL_PIN | OLED_I2C_SDA_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = OLED_I2C_SCL_PIN | OLED_I2C_SDA_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // 必须是开漏
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(OLED_I2C_GPIO_PORT, &GPIO_InitStruct);

    // 3. 拉高 SCL 和 SDA
    HAL_GPIO_WritePin(OLED_I2C_GPIO_PORT, OLED_I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(OLED_I2C_GPIO_PORT, OLED_I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    // 4. 模拟时钟脉冲，直到从机释放 SDA (SDA变成高电平)
    for(int i = 0; i < 9; i++) 
    {
        HAL_GPIO_WritePin(OLED_I2C_GPIO_PORT, OLED_I2C_SCL_PIN, GPIO_PIN_RESET); // SCL 低
        HAL_Delay(1);
        HAL_GPIO_WritePin(OLED_I2C_GPIO_PORT, OLED_I2C_SCL_PIN, GPIO_PIN_SET);   // SCL 高
        HAL_Delay(1);
        
        // 如果读到 SDA 已经被释放（拉高），就可以提前退出
        if(HAL_GPIO_ReadPin(OLED_I2C_GPIO_PORT, OLED_I2C_SDA_PIN) == GPIO_PIN_SET) {
            break; 
        }
    }
    
    // 5. 产生 STOP：SCL 为高时释放 SDA，结束从机可能残留的事务。
    HAL_GPIO_WritePin(OLED_I2C_GPIO_PORT, OLED_I2C_SCL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(OLED_I2C_GPIO_PORT, OLED_I2C_SDA_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(OLED_I2C_GPIO_PORT, OLED_I2C_SCL_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(OLED_I2C_GPIO_PORT, OLED_I2C_SDA_PIN, GPIO_PIN_SET);
    HAL_Delay(1);

    // 6. 复位 I2C1 外设，并恢复引脚的 I2C 复用功能。
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    MX_I2C1_Init();
}

// 发送一个字节到 OLED
// mode: 0 表示命令; 1 表示数据
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t buf[2];
    
    if(mode)
    {
        buf[0] = 0x40; // 写入数据寄存器
        buf[1] = dat;
    }
    else
    {
        buf[0] = 0x00; // 写入命令寄存器
        buf[1] = dat;
    }

    // 调用 STM32 HAL 硬件 I2C 发送 (超时时间设为 10ms)
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(&hi2c1, OLED_I2C_ADDRESS, buf, 2, 10);
    
    // 如果发送超时或总线报错，执行解锁程序恢复总线
    if(status != HAL_OK)
    {
        oled_i2c_sda_unlock();
    }
}

//反显函数
void OLED_ColorTurn(uint8_t i)
{
    if(i==0) OLED_WR_Byte(0xA6, OLED_CMD); //正常显示
    if(i==1) OLED_WR_Byte(0xA7, OLED_CMD); //反色显示
}

//屏幕旋转180度
void OLED_DisplayTurn(uint8_t i)
{
    if(i==0)
    {
        OLED_WR_Byte(0xC8, OLED_CMD); //正常显示
        OLED_WR_Byte(0xA1, OLED_CMD);
    }
    if(i==1)
    {
        OLED_WR_Byte(0xC0, OLED_CMD); //反转显示
        OLED_WR_Byte(0xA0, OLED_CMD);
    }
}

//坐标设置
void OLED_Set_Pos(uint8_t x, uint8_t y) 
{
    uint16_t column = (uint16_t)x + OLED_COLUMN_OFFSET;

    OLED_WR_Byte(0xb0+y, OLED_CMD);
    OLED_WR_Byte(((column & 0xf0) >> 4) | 0x10, OLED_CMD);
    OLED_WR_Byte(column & 0x0f, OLED_CMD);
}

//开启OLED显示    
void OLED_Display_On(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X14, OLED_CMD);  //DCDC ON
    OLED_WR_Byte(0XAF, OLED_CMD);  //DISPLAY ON
}

//关闭OLED显示     
void OLED_Display_Off(void)
{
    OLED_WR_Byte(0X8D, OLED_CMD);  //SET DCDC命令
    OLED_WR_Byte(0X10, OLED_CMD);  //DCDC OFF
    OLED_WR_Byte(0XAE, OLED_CMD);  //DISPLAY OFF
}
	 
//清屏函数
void OLED_Clear(void)  
{  
    uint8_t i, n;		    
    for(i = 0; i < OLED_PAGE_COUNT; i++)
    {  
        // 清屏与文字、图片共用列偏移，覆盖实际可见的全部 128 列。
        OLED_Set_Pos(0, i);
        for(n = 0; n < OLED_WIDTH; n++) OLED_WR_Byte(0, OLED_DATA);
    } 
}

//显示字符
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey)
{      	
    uint8_t c = 0, sizex = sizey / 2;
    uint16_t i = 0, size1;
    if(sizey == 8) size1 = 6;
    else size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * (sizey / 2);
    
    c = chr - ' '; // 得到偏移后的值
    OLED_Set_Pos(x, y);
    for(i = 0; i < size1; i++)
    {
        if(i % sizex == 0 && sizey != 8) OLED_Set_Pos(x, y++);
        if(sizey == 8) OLED_WR_Byte(asc2_0806[c][i], OLED_DATA);      // 6X8字号
        else if(sizey == 16) OLED_WR_Byte(asc2_1608[c][i], OLED_DATA);// 8x16字号
        else return;
    }
}

// m^n函数
uint32_t oled_pow(uint8_t m, uint8_t n)
{
    uint32_t result = 1;	 
    while(n--) result *= m;    
    return result;
}

//显示数字		  
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey)
{         	
    uint8_t t, temp, m = 0;
    uint8_t enshow = 0;
    if(sizey == 8) m = 2;
    for(t = 0; t < len; t++)
    {
        temp = (num / oled_pow(10, len - t - 1)) % 10;
        if(enshow == 0 && t < (len - 1))
        {
            if(temp == 0)
            {
                OLED_ShowChar(x + (sizey / 2 + m) * t, y, ' ', sizey);
                continue;
            }else enshow = 1;
        }
        OLED_ShowChar(x + (sizey / 2 + m) * t, y, temp + '0', sizey);
    }
}

//显示一个字符号串
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey)
{
    uint8_t j = 0;
    while (chr[j] != '\0')
    {		
        OLED_ShowChar(x, y, chr[j++], sizey);
        if(sizey == 8) x += 6;
        else x += sizey / 2;
    }
}

//显示汉字
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey)
{
    uint16_t i, size1 = (sizey / 8 + ((sizey % 8) ? 1 : 0)) * sizey;
    for(i = 0; i < size1; i++)
    {
        if(i % sizey == 0) OLED_Set_Pos(x, y++);
        if(sizey == 16) OLED_WR_Byte(Hzk[no][i], OLED_DATA);
        else return;
    }				
}

//显示图片
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[])
{ 	
    uint16_t j = 0;
    uint8_t i, m;
    sizey = sizey / 8 + ((sizey % 8) ? 1 : 0);
    for(i = 0; i < sizey; i++)
    {
        OLED_Set_Pos(x, i + y);
        for(m = 0; m < sizex; m++)
        {      
            OLED_WR_Byte(BMP[j++], OLED_DATA);	    	
        }
    }
}

//初始化SSD1306					    
void OLED_Init(void)
{
    // 如果总线死锁，先进行强制解锁
    // 调用前先执行 MX_I2C1_Init()，确保 GPIOB 时钟和 I2C1 已初始化。
    if(HAL_GPIO_ReadPin(OLED_I2C_GPIO_PORT, OLED_I2C_SDA_PIN) == GPIO_PIN_RESET) {
        oled_i2c_sda_unlock();
    }

    HAL_Delay(200);

    OLED_WR_Byte(0xAE, OLED_CMD); //--turn off oled panel
    OLED_WR_Byte(0x00, OLED_CMD); //---set low column address
    OLED_WR_Byte(0x10, OLED_CMD); //---set high column address
    OLED_WR_Byte(0x40, OLED_CMD); //--set start line address  Set Mapping RAM Display Start Line (0x00~0x3F)
    OLED_WR_Byte(0x81, OLED_CMD); //--set contrast control register
    OLED_WR_Byte(0xCF, OLED_CMD); // Set SEG Output Current Brightness
    OLED_WR_Byte(0xA1, OLED_CMD); //--Set SEG/Column Mapping     0xa0左右反置 0xa1正常
    OLED_WR_Byte(0xC8, OLED_CMD); //Set COM/Row Scan Direction   0xc0上下反置 0xc8正常
    OLED_WR_Byte(0xA6, OLED_CMD); //--set normal display
    OLED_WR_Byte(0xA8, OLED_CMD); //--set multiplex ratio(1 to 64)
    OLED_WR_Byte(OLED_HEIGHT - 1U, OLED_CMD); // 64 行为 0x3F，32 行为 0x1F
    OLED_WR_Byte(0xD3, OLED_CMD); //-set display offset	Shift Mapping RAM Counter (0x00~0x3F)
    OLED_WR_Byte(0x00, OLED_CMD); //-not offset
    OLED_WR_Byte(0xd5, OLED_CMD); //--set display clock divide ratio/oscillator frequency
    OLED_WR_Byte(0x80, OLED_CMD); //--set divide ratio, Set Clock as 100 Frames/Sec
    OLED_WR_Byte(0xD9, OLED_CMD); //--set pre-charge period
    OLED_WR_Byte(0xF1, OLED_CMD); //Set Pre-Charge as 15 Clocks & Discharge as 1 Clock
    OLED_WR_Byte(0xDA, OLED_CMD); //--set com pins hardware configuration
    OLED_WR_Byte((OLED_HEIGHT == 64U) ? 0x12 : 0x02, OLED_CMD);
    OLED_WR_Byte(0xDB, OLED_CMD); //--set vcomh
    OLED_WR_Byte(0x30, OLED_CMD); // SSD1306 支持的 VCOMH 电平：约 0.83 * VCC
    OLED_WR_Byte(0x20, OLED_CMD); //-Set Page Addressing Mode (0x00/0x01/0x02)
    OLED_WR_Byte(0x02, OLED_CMD); //
    OLED_WR_Byte(0x8D, OLED_CMD); //--set Charge Pump enable/disable
    OLED_WR_Byte(0x14, OLED_CMD); //--set(0x10) disable
    OLED_WR_Byte(0xA4, OLED_CMD); // Disable Entire Display On (0xa4/0xa5)
    OLED_WR_Byte(0xA6, OLED_CMD); // Disable Inverse Display On (0xa6/a7) 
    OLED_Clear();
    OLED_WR_Byte(0xAF, OLED_CMD); /*display ON*/ 
}
