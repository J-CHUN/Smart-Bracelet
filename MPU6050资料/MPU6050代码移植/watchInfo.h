#ifndef __WATCH_INFO_H
#define __WATCH_INFO_H

#ifdef __cplusplus
extern "C"
{
#endif

/*********************************************************************
 * INCLUDES
 */
#include "stm32f4xx.h"
#include "stepAlgorithm.h"
/*********************************************************************
 * TYPEDEFS
 */
/*********************************************************************
*  EXTERNAL VARIABLES
*/

/*********************************************************************
 * CONSTANTS
 */
//Èí¼þ°æ±¾Îª01.01.00
#define     SYS_VERSION       0x0101
#define     TEST_VERSION      0x00
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 *PUBLIC FUNCTIONS DECLARE
 */
u8 WatchInfo_init(void);
u8 WatchInfo_setUserInfo(u8 height,u8 weight);
personInfo_t * WatchInfo_getUserInfo(u8 *error);



/*********************************************************************
*********************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* __WATCH_INFO_H */
