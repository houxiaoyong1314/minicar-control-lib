#include "i2c.h"

/* ------------------------------------------------------------------------------------------------
 *                                          Constants
 * ------------------------------------------------------------------------------------------------
 */
#define I2C_ENS1            BV(6)  // I2C����ʹ��λ
#define I2C_STA             BV(5)  // ��ʼ��־λ
#define I2C_STO             BV(4)  // ֹͣ��־λ
#define I2C_SI              BV(3)  // �ж��ź�λ
#define I2C_AA              BV(2)  // ����ΪAAʱ��I2Cģ���ȷ�ϱ�־λ
#define I2C_MST_RD_BIT      BV(0)  // ����RD / WRnλ��ӻ���ַ���л����㡣

#define I2C_CLOCK_MASK      0x83   // ��ʼ����111��Ĭ��Ϊ����

/* ------------------------------------------------------------------------------------------------
 *                                           Typedefs
 * ------------------------------------------------------------------------------------------------
 */

typedef enum
{
  // I2C������Ϊ���豸ʱ�����еķ���״̬
  mstStarted   = 0x08,
  mstRepStart  = 0x10,
  mstAddrAckW  = 0x18,
  mstAddrNackW = 0x20,
  mstDataAckW  = 0x28,
  mstDataNackW = 0x30,
  mstLostArb   = 0x38,
  mstAddrAckR  = 0x40,
  mstAddrNackR = 0x48,
  mstDataAckR  = 0x50,
  mstDataNackR = 0x58,
} i2cStatus_t;

/* ------------------------------------------------------------------------------------------------
 *                                           Macros
 * ------------------------------------------------------------------------------------------------
 */

#define I2C_WRAPPER_DISABLE() st( I2CWC    =     0x00;              )
#define I2C_CLOCK_RATE(x)     st( I2CCFG  &=    ~I2C_CLOCK_MASK;    \
                                  I2CCFG  |=     x;                 )
#define I2C_SET_NACK()        st( I2CCFG &= ~I2C_AA; )
#define I2C_SET_ACK()         st( I2CCFG |=  I2C_AA; )

// ����I2C����
#define I2C_ENABLE()          st( I2CCFG |= (I2C_ENS1); )
#define I2C_DISABLE()         st( I2CCFG &= ~(I2C_ENS1); )

// ����������STA֮ǰ���SI��Ȼ������ֶ����STA��
#define I2C_STRT() st (             \
  I2CCFG &= ~I2C_SI;                \
  I2CCFG |= I2C_STA;                \
  while ((I2CCFG & I2C_SI) == 0);   \
  I2CCFG &= ~I2C_STA; \
)

// ���������SIǰ����STO
#define I2C_STOP() st (             \
  I2CCFG |= I2C_STO;                \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_STO) != 0);  \
)

// ֹͣʱ�����죬Ȼ���ڵ���ʱ��ȡ����
#define I2C_READ(_X_) st (          \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_SI) == 0);   \
  (_X_) = I2CDATA;                  \
)

// ����д�����ݣ�Ȼ��ֹͣʱ������
#define I2C_WRITE(_X_) st (         \
  I2CDATA = (_X_);                  \
  I2CCFG &= ~I2C_SI;                \
  while ((I2CCFG & I2C_SI) == 0);   \
)


/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */
 uint8 i2cAddr;  // Ŀ����豸��ַԤ�������ƶ�һ����ʹRD / WRn LSB���λ����Ϊ�㡣
//uint8 reg_addr;

/**************************************************************************************************
 * @fn          i2cMstStrt
 *
 * @brief       ��ΪI2C�����������Է���I2C������ʼ�źźʹӻ���ַ
 *
 * @param       RD_WRn -> RW ��дλ��Ҳ���Ǵӻ���ַ�����λ
 *
 * @return      ��ʼ�ź����󼰴ӻ���ַȷ��״̬����
 */
static uint8 i2cMstStrt(uint8 RD_WRn)
{
  I2C_STRT();                         // ������ʼ�ź�

  if (I2CSTAT == mstStarted)          // ��ʼ�ź��ѱ��ʹ�
  {
    I2C_WRITE(i2cAddr | RD_WRn);      // ��ߵ�i2cAddr�Ǵӻ���ַ����������һλ��RD_WRnΪ��дλ
  }

  return I2CSTAT;                     // ������ʼ�ź����󼰴ӻ���ַȷ��״̬
}


static uint8 i2cMst_reStrt(uint8 RD_WRn)
{
  I2C_STRT();                         // ������ʼ�ź�

  if (I2CSTAT == mstRepStart)          // ��ʼ�ź��ѱ��ʹ�
  {
    I2C_WRITE(i2cAddr | RD_WRn);      // ��ߵ�i2cAddr�Ǵӻ���ַ����������һλ��RD_WRnΪ��дλ
  }

  return I2CSTAT;                     // ������ʼ�ź����󼰴ӻ���ַȷ��״̬
}


/**************************************************************************************************
 * @fn          HalI2CInit
 *
 * @brief       ��ʼ��I2C������Ϊ����
 *
 * @param       address -> I2C�ӻ���ַ
 * @param       clockRate -> I2Cʱ��Ƶ��
 *
 * @return      None.
 */
void HalI2CInit(uint8 address, i2cClock_t clockRate)
{
  i2cAddr = address << 1;             // �ӻ���ַ������һλ

  I2C_WRAPPER_DISABLE();              // �ر�I2C
  I2CADDR = 0;                        // ��֧�ֶ�����ģʽ
  I2C_CLOCK_RATE(clockRate);          // ����ʱ��Ƶ��
  I2C_ENABLE();                       // ����I2C
}

uint8 HalI2CRead(uint8 len,uint8 addr,uint8 *pBuf)
{
  uint8 cnt = 0;
  uint8 stat = 0;
//write dummy
while(!stat) {

  if (i2cMstStrt(0) != mstAddrAckW)
  {
    len = 0;
  }

  I2C_WRITE(addr);
  stat = I2CSTAT;
  if (stat == mstDataAckW)	{  // �����Ѿ����ͣ�����ACK
        stat = 1;
  }
  else if (stat == mstDataNackW) {
  		I2C_STOP();  
		return 0;
  }else {
  I2C_STOP();  
 	return 0;
  }
}

stat = i2cMst_reStrt(I2C_MST_RD_BIT);

//restart 
  // ��ΪI2C�����������Է���I2C������ʼ�źźʹӻ���ַ
  // �ӻ���ַ+READ�Ѿ������䣬�ҷ���ACK
  if ( stat != mstAddrAckR)   
  {
    len = 0;
  }

  // �������һ��NACK�������ֽڶ���ACK�� 
  // ���ֻ��1���ֽڣ���ᷢ��һ��NACK�� 
  // ��ˣ����Ҫ��ȡ����1���ֽڣ�����ֻҪ����ACK��
  if (len > 1)
 {
    I2C_SET_ACK();
  }

  while (len > 0)
  {
    // ���豸�ڶ������һ���ֽں���Ҫ����NACK
    if (len == 1)
    {
      I2C_SET_NACK();
    }

    // ��I2C���߶�ȡһ���ֽ�
    I2C_READ(*pBuf++);
    cnt++;
    len--;

    if (I2CSTAT != mstDataAckR)     // �����Ѿ����գ�����ACK
    {
      if (I2CSTAT != mstDataNackR)  // �����Ѿ����գ�����NACK
      {
        // NACK�����Բ�Ҫ�������һ���ֽ�
        cnt--;
      }
      break;
    }
  }
  
  //I2C_SET_NACK();
  
  I2C_STOP();                        // ����ֹͣ�ź�

  return cnt;
}

/**************************************************************************************************
 * @fn          HalI2CWrite
 *
 * @brief       ��ΪI2C������ӻ�д����
 *
 * @param       len -> Ҫд�����ݳ���
 * @param       pBuf -> ָ�����ݻ�����pBuf��ָ�룬����д�ֽ�
 *
 * @return      �ɹ�д����ֽ���
 */
uint8 HalI2CWrite(uint8 len, uint8 *pBuf)
{
  // ��ΪI2C�����������Է���I2C������ʼ�źźʹӻ���ַ
  // �ӻ���ַ+WRITE�Ѿ������䣬�ҷ���ACK
  if (i2cMstStrt(0) != mstAddrAckW)
  {
    len = 0;
  }

  for (uint8 cnt = 0; cnt < len; cnt++)
  {
    // ��I2C����дһ���ֽ�
    I2C_WRITE(*pBuf++);

    if (I2CSTAT != mstDataAckW)     // �����Ѿ����ͣ�����ACK
    {
      if (I2CSTAT == mstDataNackW)  // �����Ѿ����ͣ�����NACK
      {
        len = cnt + 1;
      }
      else
      {
        len = cnt;
      }
      break;
    }
  }

  I2C_STOP();                         // ����ֹͣ�ź�

  return len;
}

void WriteData(uint8 Address,uint8 Dat){
  uint8 buf[2];
  buf[0] = Address;
  buf[1] = Dat;
  HalI2CWrite(2,buf);
}


void ReadData(uint8 addr, uint8 *Pbuf,uint8 Num){
  HalI2CRead(Num,addr,Pbuf);
}

/**************************************************************************************************
 * @fn          HalI2CDisable
 *
 * @brief       ��I2C�������ڷǻ״̬
 *
 * @param       None.
 *
 * @return      None.
 */
void HalI2CDisable(void)
{
  I2C_DISABLE();             // �ر�I2C���߹���
}

/*********************************************************************
*********************************************************************/
