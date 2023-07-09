static void AF_NO_REMAP (void) { }
#ifdef __HAL_AFIO_REMAP_I2C1_DISABLE
static void AF__HAL_AFIO_REMAP_I2C1_DISABLE(void) { __HAL_AFIO_REMAP_I2C1_DISABLE(); }
#else
static void AF__HAL_AFIO_REMAP_I2C1_DISABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_I2C1_ENABLE
static void AF__HAL_AFIO_REMAP_I2C1_ENABLE(void) { __HAL_AFIO_REMAP_I2C1_ENABLE(); }
#else
static void AF__HAL_AFIO_REMAP_I2C1_ENABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_SPI1_DISABLE
static void AF__HAL_AFIO_REMAP_SPI1_DISABLE(void) { __HAL_AFIO_REMAP_SPI1_DISABLE(); }
#else
static void AF__HAL_AFIO_REMAP_SPI1_DISABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_SPI1_ENABLE
static void AF__HAL_AFIO_REMAP_SPI1_ENABLE(void) { __HAL_AFIO_REMAP_SPI1_ENABLE(); }
#else
static void AF__HAL_AFIO_REMAP_SPI1_ENABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_TIM1_DISABLE
static void AF__HAL_AFIO_REMAP_TIM1_DISABLE(void) { __HAL_AFIO_REMAP_TIM1_DISABLE(); }
#else
static void AF__HAL_AFIO_REMAP_TIM1_DISABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_TIM1_ENABLE
static void AF__HAL_AFIO_REMAP_TIM1_ENABLE(void) { __HAL_AFIO_REMAP_TIM1_ENABLE(); }
#else
static void AF__HAL_AFIO_REMAP_TIM1_ENABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_TIM1_PARTIAL
static void AF__HAL_AFIO_REMAP_TIM1_PARTIAL(void) { __HAL_AFIO_REMAP_TIM1_PARTIAL(); }
#else
static void AF__HAL_AFIO_REMAP_TIM1_PARTIAL(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_TIM2_DISABLE
static void AF__HAL_AFIO_REMAP_TIM2_DISABLE(void) { __HAL_AFIO_REMAP_TIM2_DISABLE(); }
#else
static void AF__HAL_AFIO_REMAP_TIM2_DISABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_TIM2_PARTIAL_1
static void AF__HAL_AFIO_REMAP_TIM2_PARTIAL_1(void) { __HAL_AFIO_REMAP_TIM2_PARTIAL_1(); }
#else
static void AF__HAL_AFIO_REMAP_TIM2_PARTIAL_1(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_TIM2_PARTIAL_2
static void AF__HAL_AFIO_REMAP_TIM2_PARTIAL_2(void) { __HAL_AFIO_REMAP_TIM2_PARTIAL_2(); }
#else
static void AF__HAL_AFIO_REMAP_TIM2_PARTIAL_2(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_TIM3_DISABLE
static void AF__HAL_AFIO_REMAP_TIM3_DISABLE(void) { __HAL_AFIO_REMAP_TIM3_DISABLE(); }
#else
static void AF__HAL_AFIO_REMAP_TIM3_DISABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_TIM3_ENABLE
static void AF__HAL_AFIO_REMAP_TIM3_ENABLE(void) { __HAL_AFIO_REMAP_TIM3_ENABLE(); }
#else
static void AF__HAL_AFIO_REMAP_TIM3_ENABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_TIM3_PARTIAL
static void AF__HAL_AFIO_REMAP_TIM3_PARTIAL(void) { __HAL_AFIO_REMAP_TIM3_PARTIAL(); }
#else
static void AF__HAL_AFIO_REMAP_TIM3_PARTIAL(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_TIM4_DISABLE
static void AF__HAL_AFIO_REMAP_TIM4_DISABLE(void) { __HAL_AFIO_REMAP_TIM4_DISABLE(); }
#else
static void AF__HAL_AFIO_REMAP_TIM4_DISABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_TIM4_ENABLE
static void AF__HAL_AFIO_REMAP_TIM4_ENABLE(void) { __HAL_AFIO_REMAP_TIM4_ENABLE(); }
#else
static void AF__HAL_AFIO_REMAP_TIM4_ENABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_USART1_DISABLE
static void AF__HAL_AFIO_REMAP_USART1_DISABLE(void) { __HAL_AFIO_REMAP_USART1_DISABLE(); }
#else
static void AF__HAL_AFIO_REMAP_USART1_DISABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_USART1_ENABLE
static void AF__HAL_AFIO_REMAP_USART1_ENABLE(void) { __HAL_AFIO_REMAP_USART1_ENABLE(); }
#else
static void AF__HAL_AFIO_REMAP_USART1_ENABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_USART2_DISABLE
static void AF__HAL_AFIO_REMAP_USART2_DISABLE(void) { __HAL_AFIO_REMAP_USART2_DISABLE(); }
#else
static void AF__HAL_AFIO_REMAP_USART2_DISABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_USART2_ENABLE
static void AF__HAL_AFIO_REMAP_USART2_ENABLE(void) { __HAL_AFIO_REMAP_USART2_ENABLE(); }
#else
static void AF__HAL_AFIO_REMAP_USART2_ENABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_USART3_DISABLE
static void AF__HAL_AFIO_REMAP_USART3_DISABLE(void) { __HAL_AFIO_REMAP_USART3_DISABLE(); }
#else
static void AF__HAL_AFIO_REMAP_USART3_DISABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_USART3_ENABLE
static void AF__HAL_AFIO_REMAP_USART3_ENABLE(void) { __HAL_AFIO_REMAP_USART3_ENABLE(); }
#else
static void AF__HAL_AFIO_REMAP_USART3_ENABLE(void) { AF_NO_REMAP(); }
#endif

#ifdef __HAL_AFIO_REMAP_USART3_PARTIAL
static void AF__HAL_AFIO_REMAP_USART3_PARTIAL(void) { __HAL_AFIO_REMAP_USART3_PARTIAL(); }
#else
static void AF__HAL_AFIO_REMAP_USART3_PARTIAL(void) { AF_NO_REMAP(); }
#endif

