/*
1. 주소 찾기

#define RCC             0x40023800
#define RCC_AHBENABLE   0x30

#define GPIOC           0x40020800
#define GPIOx_MODER     0x00
#define GPIOx_OTYPER    0x04
#define GPIOx_OSPEEDR   0x08
#define GPIOx_PUPDR     0x0c
#define GPIOx_IDR       0x10
#define GPIOx_ODR       0x14

*/

//2. 주소로 변환 -------------------------------------------------------

#define RCC             0x40023800  
#define RCC_AHBENABLE   *((unsigned int*)(RCC + 0x30)) // 0x40023830

#define GPIOC           0x40020800
#define GPIOC_MODER     *((unsigned int*)(GPIOC + 0x00)) //0x40020800
#define GPIOC_OTYPER    *((unsigned int*)(GPIOC + 0x04)) //0x40020804
#define GPIOC_OSPEEDR   *((unsigned int*)(GPIOC + 0x08)) //0x40020808
#define GPIOC_PUPDR     *((unsigned int*)(GPIOC + 0x0c)) //0x4002080c
#define GPIOC_IDR       *((unsigned int*)(GPIOC + 0x10)) //0x40020810
#define GPIOC_ODR       *((unsigned int*)(GPIOC + 0x14)) //0x40020814

//---------------------------------------------------------------------



static void Delay(const unsigned int count)
{
  unsigned int i;
  for(i=(5000*count); i!=0; i--);
}

int main()
{
    RCC_AHBENABLE = 0x00000004;         //PORT clock enable
    GPIOC_MODER   = 0x00000055;         //PORT[3:0] output
    GPIOC_OTYPER  = 0x00000000;         //push-pull
    GPIOC_OSPEEDR = 0x00000000;         //speed
    GPIOC_PUPDR   = 0x00000000;         //no pull-up/pull-down
    
    while(1)
    {
        GPIOC_ODR = 0x0005; // 0,2 on  1,3 off
        Delay(1000);
        GPIOC_ODR = 0x000A; // 1,3 on  0,2 off
        Delay(1000);
    }

}



