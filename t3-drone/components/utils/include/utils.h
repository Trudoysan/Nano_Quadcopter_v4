

#ifndef COMPONENTS_PLATFORM_UTILS_H_
#define COMPONENTS_PLATFORM_UTILS_H_

#ifndef FALSE
# define FALSE 0
#endif

#ifndef TRUE
# define TRUE 1
#endif

#define pdFALSE			( ( BaseType_t ) 0 )
#define pdTRUE			( ( BaseType_t ) 1 )

#define M2T(X) ((unsigned int)(X)/ portTICK_PERIOD_MS) //ms to tick
#define F2T(X) ((unsigned int)((configTICK_RATE_HZ/(X))))
#define T2M(X) ((unsigned int)(X)* portTICK_PERIOD_MS)   //tick to ms

// Seconds to OS ticks
#define S2T(X) ((portTickType)((X) * configTICK_RATE_HZ))
#define T2S(X) ((X) / (float)configTICK_RATE_HZ)


#endif /* COMPONENTS_PLATFORM_UTILS_H_ */
