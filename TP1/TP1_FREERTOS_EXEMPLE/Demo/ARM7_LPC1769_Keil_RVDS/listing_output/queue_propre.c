#include <stdlib.h>
#include <string.h>

/* Defining MPU_WRAPPERS_INCLUDED_FROM_API_FILE prevents task.h from redefining
all the API functions to use the MPU wrappers.  That should only be done when
task.h is included from an application file. */
#define MPU_WRAPPERS_INCLUDED_FROM_API_FILE

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

typedef struct QueuePointers
{
	int8_t *pcTail;					 
	int8_t *pcReadFrom;				 
} QueuePointers_t;
typedef struct SemaphoreData
{
	TaskHandle_t xMutexHolder;		  
	UBaseType_t uxRecursiveCallCount; 
} SemaphoreData_t;
typedef struct QueueDefinition 		 
{
	int8_t *pcHead;					 
	int8_t *pcWriteTo;				 
	union
	{	QueuePointers_t xQueue;		 
		SemaphoreData_t xSemaphore;  
	} u;
	List_t xTasksWaitingToSend;		 
	List_t xTasksWaitingToReceive;	 
	volatile UBaseType_t uxMessagesWaiting; 
	UBaseType_t uxLength;			 
	UBaseType_t uxItemSize;			 
	volatile int8_t cRxLock;		 
	volatile int8_t cTxLock;		 
} xQUEUE;
typedef xQUEUE Queue_t;
static void prvUnlockQueue( Queue_t * const pxQueue ) ;
static BaseType_t prvIsQueueEmpty( const Queue_t *pxQueue ) ;
static BaseType_t prvIsQueueFull( const Queue_t *pxQueue ) ;
static BaseType_t prvCopyDataToQueue( Queue_t * const pxQueue, const void *pvItemToQueue, const BaseType_t xPosition ) ;
static void prvCopyDataFromQueue( Queue_t * const pxQueue, void * const pvBuffer ) ;
static void prvInitialiseNewQueue( const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, uint8_t *pucQueueStorage, const uint8_t ucQueueType, Queue_t *pxNewQueue ) ;
//**************************************************************************************
BaseType_t xQueueGenericReset( QueueHandle_t xQueue, BaseType_t xNewQueue )
{  Queue_t * const pxQueue = xQueue;
	vPortEnterCritical();
	{   pxQueue->u.xQueue.pcTail = pxQueue->pcHead + ( pxQueue->uxLength * pxQueue->uxItemSize );  
		pxQueue->uxMessagesWaiting = ( UBaseType_t ) 0U;
		pxQueue->pcWriteTo = pxQueue->pcHead;
		pxQueue->u.xQueue.pcReadFrom = pxQueue->pcHead + ( ( pxQueue->uxLength - 1U ) * pxQueue->uxItemSize );  
		pxQueue->cRxLock = ( ( int8_t ) -1 );
		pxQueue->cTxLock = ( ( int8_t ) -1 );
		if( xNewQueue == ( ( BaseType_t ) 0 ) )
		{if( ( ( ( &( pxQueue ->xTasksWaitingToSend ) )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) == ( ( BaseType_t ) 0 ) )
			{if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToSend ) ) != ( ( BaseType_t ) 0 ) )
				{ queueYIELD_IF_USING_PREEMPTION(); }
			}			
		}
		else
		{	vListInitialise( &( pxQueue->xTasksWaitingToSend ) );
			vListInitialise( &( pxQueue->xTasksWaitingToReceive ) );
		}
	}
	vPortExitCritical();
	return ( ( ( BaseType_t ) 1 ) );
}
//*************************************************************************************************
QueueHandle_t xQueueGenericCreate( const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, const uint8_t ucQueueType )
	{Queue_t *pxNewQueue;
	 size_t xQueueSizeInBytes;
	 uint8_t *pucQueueStorage;
		if( uxItemSize == ( UBaseType_t ) 0 )	{xQueueSizeInBytes = ( size_t ) 0;	}
		else 		                            {xQueueSizeInBytes = ( size_t ) ( uxQueueLength * uxItemSize );	}
		pxNewQueue = ( Queue_t * ) pvPortMalloc( sizeof( Queue_t ) + xQueueSizeInBytes );  
		if( pxNewQueue != 0 )
		{   pucQueueStorage = ( uint8_t * ) pxNewQueue;
			pucQueueStorage += sizeof( Queue_t );  
			prvInitialiseNewQueue( uxQueueLength, uxItemSize, pucQueueStorage, ucQueueType, pxNewQueue );
		}		
	 return pxNewQueue;
	}
//************************************************************************************************************
	
static void prvInitialiseNewQueue( const UBaseType_t uxQueueLength, const UBaseType_t uxItemSize, uint8_t *pucQueueStorage, const uint8_t ucQueueType, Queue_t *pxNewQueue )
{
	( void ) ucQueueType;
	if( uxItemSize == ( UBaseType_t ) 0 )
	{/* No RAM was allocated for the queue storage area, but PC head cannot
		be set to NULL because NULL is used as a key to say the queue is used as
		a mutex.  Therefore just set pcHead to point to the queue as a benign
		value that is known to be within the memory map. */
		pxNewQueue->pcHead = ( int8_t * ) pxNewQueue;	}
	else
	{pxNewQueue->pcHead = ( int8_t * ) pucQueueStorage;	}
	pxNewQueue->uxLength = uxQueueLength;
	pxNewQueue->uxItemSize = uxItemSize;
	( void ) xQueueGenericReset( pxNewQueue, ( ( BaseType_t ) 1 ) );
	;
}
//**************************************************************************************
BaseType_t xQueueGenericSend( QueueHandle_t xQueue, const void * const pvItemToQueue, TickType_t xTicksToWait, const BaseType_t xCopyPosition )
{
BaseType_t xEntryTimeSet = ( ( BaseType_t ) 0 ), xYieldRequired;
TimeOut_t xTimeOut;
Queue_t * const pxQueue = xQueue;
	for( ;; )
	{vPortEnterCritical();
		if( ( pxQueue->uxMessagesWaiting < pxQueue->uxLength ) || ( xCopyPosition == ( ( BaseType_t ) 2 ) ) )
			{xYieldRequired = prvCopyDataToQueue( pxQueue, pvItemToQueue, xCopyPosition );
					if( ( ( ( &( pxQueue ->xTasksWaitingToReceive ) )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) == ( ( BaseType_t ) 0 ) )
					{if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToReceive ) ) != ( ( BaseType_t ) 0 ) )
						{ queueYIELD_IF_USING_PREEMPTION(); }
					}
					else if( xYieldRequired != ( ( BaseType_t ) 0 ) ){queueYIELD_IF_USING_PREEMPTION(); 	}
	       vPortExitCritical();
				return ( ( ( BaseType_t ) 1 ) );
			}
			else
			{if( xTicksToWait == ( TickType_t ) 0 )
				{vPortExitCritical();
                 return ( ( BaseType_t ) 0 );
				}
				else if( xEntryTimeSet == ( ( BaseType_t ) 0 ) )
				{	vTaskInternalSetTimeOutState( &xTimeOut );
					xEntryTimeSet = ( ( BaseType_t ) 1 );
				}				
			}
	vPortExitCritical();
	vTaskSuspendAll();
	vPortEnterCritical(); 
	  if( ( pxQueue )->cRxLock == ( ( int8_t ) -1 ) ) { ( pxQueue )->cRxLock = ( ( int8_t ) 0 ); } 
	  if( ( pxQueue )->cTxLock == ( ( int8_t ) -1 ) ) { ( pxQueue )->cTxLock = ( ( int8_t ) 0 ); } 	 
	vPortExitCritical();
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == ( ( BaseType_t ) 0 ) )
		{if( prvIsQueueFull( pxQueue ) != ( ( BaseType_t ) 0 ) )
			{	vTaskPlaceOnEventList( &( pxQueue->xTasksWaitingToSend ), xTicksToWait );
				prvUnlockQueue( pxQueue );
				if( xTaskResumeAll() == ( ( BaseType_t ) 0 ) )	{queueYIELD_IF_USING_PREEMPTION(); 		}
			}
			else
			{   prvUnlockQueue( pxQueue );
				( void ) xTaskResumeAll();
			}
		}
		else
		{prvUnlockQueue( pxQueue );
			( void ) xTaskResumeAll();
			return ( ( BaseType_t ) 0 );
		}
	}  
}
//*********************************************************************************
BaseType_t xQueueGenericSendFromISR( QueueHandle_t xQueue, const void * const pvItemToQueue, BaseType_t * const pxHigherPriorityTaskWoken, const BaseType_t xCopyPosition )
{
BaseType_t xReturn;
UBaseType_t uxSavedInterruptStatus;
Queue_t * const pxQueue = xQueue;
	uxSavedInterruptStatus = ulPortRaiseBASEPRI();
	{if( ( pxQueue->uxMessagesWaiting < pxQueue->uxLength ) || ( xCopyPosition == ( ( BaseType_t ) 2 ) ) )
		{	const int8_t cTxLock = pxQueue->cTxLock;
			( void ) prvCopyDataToQueue( pxQueue, pvItemToQueue, xCopyPosition );
			if( cTxLock == ( ( int8_t ) -1 ) )
			{	if( ( ( ( &( pxQueue ->xTasksWaitingToReceive ) )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) == ( ( BaseType_t ) 0 ) )
					{if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToReceive ) ) != ( ( BaseType_t ) 0 ) )
						{if( pxHigherPriorityTaskWoken != 0 )	{	*pxHigherPriorityTaskWoken = ( ( BaseType_t ) 1 ); 	}							
						}						
					}									
			}
			else 	{	pxQueue->cTxLock = ( int8_t ) ( cTxLock + 1 );	}
			xReturn = ( ( ( BaseType_t ) 1 ) );
		}
		else
		{	xReturn = ( ( BaseType_t ) 0 );	}
	}
	vPortSetBASEPRI(uxSavedInterruptStatus);
	return xReturn;
}
//*****************************************************************************************************
BaseType_t xQueueGiveFromISR( QueueHandle_t xQueue, BaseType_t * const pxHigherPriorityTaskWoken )
{
BaseType_t xReturn;
UBaseType_t uxSavedInterruptStatus;
Queue_t * const pxQueue = xQueue;
	uxSavedInterruptStatus = ulPortRaiseBASEPRI();
	{const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;
		if( uxMessagesWaiting < pxQueue->uxLength )
		{	const int8_t cTxLock = pxQueue->cTxLock;
			pxQueue->uxMessagesWaiting = uxMessagesWaiting + ( UBaseType_t ) 1;
			if( cTxLock == ( ( int8_t ) -1 ) )
			{	if( ( ( ( &( pxQueue ->xTasksWaitingToReceive ) )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) == ( ( BaseType_t ) 0 ) )
					{if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToReceive ) ) != ( ( BaseType_t ) 0 ) )
						{if( pxHigherPriorityTaskWoken != 0 )	{*pxHigherPriorityTaskWoken = ( ( BaseType_t ) 1 );	}						
						}					
					}
			}
			else
			{	pxQueue->cTxLock = ( int8_t ) ( cTxLock + 1 );}
			xReturn = ( ( ( BaseType_t ) 1 ) );
		}
		else	{xReturn = ( ( BaseType_t ) 0 );}
	}
	vPortSetBASEPRI(uxSavedInterruptStatus);
	return xReturn;
}
//***************************************************************************************************
BaseType_t xQueueReceive( QueueHandle_t xQueue, void * const pvBuffer, TickType_t xTicksToWait )
{
BaseType_t xEntryTimeSet = ( ( BaseType_t ) 0 );
TimeOut_t xTimeOut;
Queue_t * const pxQueue = xQueue;
	for( ;; )
	{vPortEnterCritical();
		const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;
			if( uxMessagesWaiting > ( UBaseType_t ) 0 )
			{   prvCopyDataFromQueue( pxQueue, pvBuffer );
				pxQueue->uxMessagesWaiting = uxMessagesWaiting - ( UBaseType_t ) 1;
				if( ( ( ( &( pxQueue ->xTasksWaitingToSend ) )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) == ( ( BaseType_t ) 0 ) )
				{if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToSend ) ) != ( ( BaseType_t ) 0 ) )
					{queueYIELD_IF_USING_PREEMPTION();}				
				}				
				vPortExitCritical();
				return ( ( ( BaseType_t ) 1 ) );
			}
			else
			{  if( xTicksToWait == ( TickType_t ) 0 )
				{   vPortExitCritical();
					return ( ( BaseType_t ) 0 );
				}
				else if( xEntryTimeSet == ( ( BaseType_t ) 0 ) )
				{   vTaskInternalSetTimeOutState( &xTimeOut );
					xEntryTimeSet = ( ( BaseType_t ) 1 );
				}			
			}
	vPortExitCritical();
	vTaskSuspendAll();
	vPortEnterCritical();
		  if( ( pxQueue )->cRxLock == ( ( int8_t ) -1 ) )  { ( pxQueue )->cRxLock = ( ( int8_t ) 0 ); } 
		  if( ( pxQueue )->cTxLock == ( ( int8_t ) -1 ) )  { ( pxQueue )->cTxLock = ( ( int8_t ) 0 ); } 		  
	vPortExitCritical();
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == ( ( BaseType_t ) 0 ) )
		{  if( prvIsQueueEmpty( pxQueue ) != ( ( BaseType_t ) 0 ) )
			{  vTaskPlaceOnEventList( &( pxQueue->xTasksWaitingToReceive ), xTicksToWait );
				prvUnlockQueue( pxQueue );
				if( xTaskResumeAll() == ( ( BaseType_t ) 0 ) )	{queueYIELD_IF_USING_PREEMPTION(); 		}				
			}
			else
			{	prvUnlockQueue( pxQueue );
				( void ) xTaskResumeAll();
			}
		}
		else
		{  prvUnlockQueue( pxQueue );
			( void ) xTaskResumeAll();
			if( prvIsQueueEmpty( pxQueue ) != ( ( BaseType_t ) 0 ) )	{return ( ( BaseType_t ) 0 );}	
		}
	}  
}

//*************************************************************************************************
BaseType_t xQueueSemaphoreTake( QueueHandle_t xQueue, TickType_t xTicksToWait )
{
BaseType_t xEntryTimeSet = ( ( BaseType_t ) 0 );
TimeOut_t xTimeOut;
Queue_t * const pxQueue = xQueue;
	for( ;; )
	{ vPortEnterCritical();
		{const UBaseType_t uxSemaphoreCount = pxQueue->uxMessagesWaiting;
			if( uxSemaphoreCount > ( UBaseType_t ) 0 )
			{	pxQueue->uxMessagesWaiting = uxSemaphoreCount - ( UBaseType_t ) 1;
				if( ( ( ( &( pxQueue ->xTasksWaitingToSend ) )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) == ( ( BaseType_t ) 0 ) )
				{ if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToSend ) ) != ( ( BaseType_t ) 0 ) )
					{ queueYIELD_IF_USING_PREEMPTION(); }					
				}				
				vPortExitCritical();
				return ( ( ( BaseType_t ) 1 ) );
			}
			else
			{  if( xTicksToWait == ( TickType_t ) 0 )
				{   vPortExitCritical();
					return ( ( BaseType_t ) 0 );
				}
				else if( xEntryTimeSet == ( ( BaseType_t ) 0 ) )
				{	vTaskInternalSetTimeOutState( &xTimeOut );
					xEntryTimeSet = ( ( BaseType_t ) 1 );
				}			
			}
		}
		vPortExitCritical();
		vTaskSuspendAll();
		vPortEnterCritical(); { if( ( pxQueue )->cRxLock == ( ( int8_t ) -1 ) ) { ( pxQueue )->cRxLock = ( ( int8_t ) 0 ); } if( ( pxQueue )->cTxLock == ( ( int8_t ) -1 ) ) { ( pxQueue )->cTxLock = ( ( int8_t ) 0 ); } } vPortExitCritical();
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == ( ( BaseType_t ) 0 ) )
		{  if( prvIsQueueEmpty( pxQueue ) != ( ( BaseType_t ) 0 ) )
			{	vTaskPlaceOnEventList( &( pxQueue->xTasksWaitingToReceive ), xTicksToWait );
				prvUnlockQueue( pxQueue );
				if( xTaskResumeAll() == ( ( BaseType_t ) 0 ) )	{queueYIELD_IF_USING_PREEMPTION(); 	}			
			}
			else
			{	prvUnlockQueue( pxQueue );
				( void ) xTaskResumeAll();
			}
		}
		else
		{   prvUnlockQueue( pxQueue );
			( void ) xTaskResumeAll();
			if( prvIsQueueEmpty( pxQueue ) != ( ( BaseType_t ) 0 ) )   {	return ( ( BaseType_t ) 0 );}			
		}
	}  
}
//********************************************************************************************************
BaseType_t xQueuePeek( QueueHandle_t xQueue, void * const pvBuffer, TickType_t xTicksToWait )
{
BaseType_t xEntryTimeSet = ( ( BaseType_t ) 0 );
TimeOut_t xTimeOut;
int8_t *pcOriginalReadPosition;
Queue_t * const pxQueue = xQueue;
	for( ;; )
	{	vPortEnterCritical();		
			const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;
			if( uxMessagesWaiting > ( UBaseType_t ) 0 )
			{   pcOriginalReadPosition = pxQueue->u.xQueue.pcReadFrom;
				prvCopyDataFromQueue( pxQueue, pvBuffer );
				pxQueue->u.xQueue.pcReadFrom = pcOriginalReadPosition;
				if( ( ( ( &( pxQueue ->xTasksWaitingToReceive ) )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) == ( ( BaseType_t ) 0 ) )
				{if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToReceive ) ) != ( ( BaseType_t ) 0 ) )
					{ queueYIELD_IF_USING_PREEMPTION();	}					
				}				
				vPortExitCritical();
				return ( ( ( BaseType_t ) 1 ) );
			}
			else
			{
				if( xTicksToWait == ( TickType_t ) 0 )
				{	vPortExitCritical();
					return ( ( BaseType_t ) 0 );
				}
				else if( xEntryTimeSet == ( ( BaseType_t ) 0 ) )
				{	vTaskInternalSetTimeOutState( &xTimeOut );
					xEntryTimeSet = ( ( BaseType_t ) 1 );
				}			
			}
		vPortExitCritical();
		vTaskSuspendAll();
		vPortEnterCritical();
		  if( ( pxQueue )->cRxLock == ( ( int8_t ) -1 ) ) { ( pxQueue )->cRxLock = ( ( int8_t ) 0 ); } 
		   if( ( pxQueue )->cTxLock == ( ( int8_t ) -1 ) ) { ( pxQueue )->cTxLock = ( ( int8_t ) 0 ); } 		   
	    vPortExitCritical();
		if( xTaskCheckForTimeOut( &xTimeOut, &xTicksToWait ) == ( ( BaseType_t ) 0 ) )
		{if( prvIsQueueEmpty( pxQueue ) != ( ( BaseType_t ) 0 ) )
			{	vTaskPlaceOnEventList( &( pxQueue->xTasksWaitingToReceive ), xTicksToWait );
				prvUnlockQueue( pxQueue );
				if( xTaskResumeAll() == ( ( BaseType_t ) 0 ) )	{queueYIELD_IF_USING_PREEMPTION(); 	}				
			}
			else
			{  prvUnlockQueue( pxQueue );
			   ( void ) xTaskResumeAll();
			}
		}
		else
		{	prvUnlockQueue( pxQueue );
			( void ) xTaskResumeAll();
			if( prvIsQueueEmpty( pxQueue ) != ( ( BaseType_t ) 0 ) ){	return ( ( BaseType_t ) 0 );}			
		}
	}  
}
//****************************************************************************************************************************
BaseType_t xQueueReceiveFromISR( QueueHandle_t xQueue, void * const pvBuffer, BaseType_t * const pxHigherPriorityTaskWoken )
{BaseType_t xReturn;
UBaseType_t uxSavedInterruptStatus;
Queue_t * const pxQueue = xQueue;
	uxSavedInterruptStatus = ulPortRaiseBASEPRI();
	{const UBaseType_t uxMessagesWaiting = pxQueue->uxMessagesWaiting;
		if( uxMessagesWaiting > ( UBaseType_t ) 0 )
		{const int8_t cRxLock = pxQueue->cRxLock;
			prvCopyDataFromQueue( pxQueue, pvBuffer );
			pxQueue->uxMessagesWaiting = uxMessagesWaiting - ( UBaseType_t ) 1;
			if( cRxLock == ( ( int8_t ) -1 ) )
			{if( ( ( ( &( pxQueue ->xTasksWaitingToSend ) )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) == ( ( BaseType_t ) 0 ) )
				{if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToSend ) ) != ( ( BaseType_t ) 0 ) )
					{if( pxHigherPriorityTaskWoken != 0 )
						{*pxHigherPriorityTaskWoken = ( ( BaseType_t ) 1 );	}					
					}					
				}				
			}
			else  {	pxQueue->cRxLock = ( int8_t ) ( cRxLock + 1 ); }
			xReturn = ( ( ( BaseType_t ) 1 ) );
		}
		else 	{ xReturn = ( ( ( BaseType_t ) 0 ) );}
	}
	vPortSetBASEPRI(uxSavedInterruptStatus);
	return xReturn;
}
//*******************************************************************************************************
BaseType_t xQueuePeekFromISR( QueueHandle_t xQueue,  void * const pvBuffer )
{BaseType_t xReturn;
UBaseType_t uxSavedInterruptStatus;
int8_t *pcOriginalReadPosition;
Queue_t * const pxQueue = xQueue;
	uxSavedInterruptStatus = ulPortRaiseBASEPRI();
		if( pxQueue->uxMessagesWaiting > ( UBaseType_t ) 0 )
		{  pcOriginalReadPosition = pxQueue->u.xQueue.pcReadFrom;
			prvCopyDataFromQueue( pxQueue, pvBuffer );
			pxQueue->u.xQueue.pcReadFrom = pcOriginalReadPosition;
			xReturn = ( ( ( BaseType_t ) 1 ) );
		}
		else		{	xReturn = ( ( ( BaseType_t ) 0 ) );		}
	vPortSetBASEPRI(uxSavedInterruptStatus);
	return xReturn;
}
//**********************************************************************************************
UBaseType_t uxQueueMessagesWaiting( const QueueHandle_t xQueue )
{UBaseType_t uxReturn;
	vPortEnterCritical();
		uxReturn = ( ( Queue_t * ) xQueue )->uxMessagesWaiting;
	vPortExitCritical();
	return uxReturn;
}  
//**************************************************************************************************
UBaseType_t uxQueueSpacesAvailable( const QueueHandle_t xQueue )
{ UBaseType_t uxReturn;
  Queue_t * const pxQueue = xQueue;
	vPortEnterCritical();
		uxReturn = pxQueue->uxLength - pxQueue->uxMessagesWaiting;
	vPortExitCritical();
	return uxReturn;
} 
//******************************************************************************************************* 
UBaseType_t uxQueueMessagesWaitingFromISR( const QueueHandle_t xQueue )
{ UBaseType_t uxReturn;
  Queue_t * const pxQueue = xQueue;
	uxReturn = pxQueue->uxMessagesWaiting;
	return uxReturn;
}  
//********************************************************************************************************
void vQueueDelete( QueueHandle_t xQueue )
{ Queue_t * const pxQueue = xQueue;
  vPortFree( pxQueue );
}
//**********************************************************************************************************
static BaseType_t prvCopyDataToQueue( Queue_t * const pxQueue, const void *pvItemToQueue, const BaseType_t xPosition )
{ BaseType_t xReturn = ( ( BaseType_t ) 0 );
  UBaseType_t uxMessagesWaiting;
	uxMessagesWaiting = pxQueue->uxMessagesWaiting;
	if( pxQueue->uxItemSize == ( UBaseType_t ) 0 )
	{
	}
	else if( xPosition == ( ( BaseType_t ) 0 ) )
	{
		( void ) memcpy( ( void * ) pxQueue->pcWriteTo, pvItemToQueue, ( size_t ) pxQueue->uxItemSize );  
		pxQueue->pcWriteTo += pxQueue->uxItemSize;  
		if( pxQueue->pcWriteTo >= pxQueue->u.xQueue.pcTail )  
		{
			pxQueue->pcWriteTo = pxQueue->pcHead;
		}
		
	}
	else
	{
		( void ) memcpy( ( void * ) pxQueue->u.xQueue.pcReadFrom, pvItemToQueue, ( size_t ) pxQueue->uxItemSize );  
		pxQueue->u.xQueue.pcReadFrom -= pxQueue->uxItemSize;
		if( pxQueue->u.xQueue.pcReadFrom < pxQueue->pcHead )  
		{
			pxQueue->u.xQueue.pcReadFrom = ( pxQueue->u.xQueue.pcTail - pxQueue->uxItemSize );
		}
		if( xPosition == ( ( BaseType_t ) 2 ) )
		{
			if( uxMessagesWaiting > ( UBaseType_t ) 0 )
			{
				--uxMessagesWaiting;
			}			
		}		
	}
	pxQueue->uxMessagesWaiting = uxMessagesWaiting + ( UBaseType_t ) 1;
	return xReturn;
}
//**********************************************************************************************************
static void prvCopyDataFromQueue( Queue_t * const pxQueue, void * const pvBuffer )
{if( pxQueue->uxItemSize != ( UBaseType_t ) 0 )
	{
		pxQueue->u.xQueue.pcReadFrom += pxQueue->uxItemSize;  
		if( pxQueue->u.xQueue.pcReadFrom >= pxQueue->u.xQueue.pcTail )  
		{
			pxQueue->u.xQueue.pcReadFrom = pxQueue->pcHead;
		}
		else
		{
			;
		}
		( void ) memcpy( ( void * ) pvBuffer, ( void * ) pxQueue->u.xQueue.pcReadFrom, ( size_t ) pxQueue->uxItemSize );  
	}
}
//***************************************************************************************************************
static void prvUnlockQueue( Queue_t * const pxQueue )
{   vPortEnterCritical();
	{
		int8_t cTxLock = pxQueue->cTxLock;
		while( cTxLock > ( ( int8_t ) 0 ) )
		{	if( ( ( ( &( pxQueue ->xTasksWaitingToReceive ) )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) == ( ( BaseType_t ) 0 ) )
				{if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToReceive ) ) != ( ( BaseType_t ) 0 ) )
					{	vTaskMissedYield();		}
				}
				else	{	break;	}		
			--cTxLock;
		}
		pxQueue->cTxLock = ( ( int8_t ) -1 );
	}
	vPortExitCritical();
	vPortEnterCritical();
	{int8_t cRxLock = pxQueue->cRxLock;
		while( cRxLock > ( ( int8_t ) 0 ) )
		{if( ( ( ( &( pxQueue ->xTasksWaitingToSend ) )->uxNumberOfItems == ( UBaseType_t ) 0 ) ? ( ( BaseType_t ) 1 ) : ( ( BaseType_t ) 0 ) ) == ( ( BaseType_t ) 0 ) )
			{ if( xTaskRemoveFromEventList( &( pxQueue->xTasksWaitingToSend ) ) != ( ( BaseType_t ) 0 ) )
				{	vTaskMissedYield();	}			
				--cRxLock;
			}
			else {	break;	}
		}
		pxQueue->cRxLock = ( ( int8_t ) -1 );
	}
	vPortExitCritical();
}
//****************************************************************************************************************
static BaseType_t prvIsQueueEmpty( const Queue_t *pxQueue )
{   BaseType_t xReturn;
	vPortEnterCritical();
	if( pxQueue->uxMessagesWaiting == ( UBaseType_t ) 0 )	{xReturn = ( ( BaseType_t ) 1 );}
	else                                                	{xReturn = ( ( BaseType_t ) 0 );}
	vPortExitCritical();
	return xReturn;
}
//**********************************************************************************************************************
BaseType_t xQueueIsQueueEmptyFromISR( const QueueHandle_t xQueue )
{ BaseType_t xReturn;
  Queue_t * const pxQueue = xQueue;
	if( pxQueue->uxMessagesWaiting == ( UBaseType_t ) 0 )	{xReturn = ( ( BaseType_t ) 1 );}
	else                                                	{xReturn = ( ( BaseType_t ) 0 );}
	return xReturn;
}  
//*************************************************************************************************************************
static BaseType_t prvIsQueueFull( const Queue_t *pxQueue )
{ BaseType_t xReturn;
	vPortEnterCritical();
		if( pxQueue->uxMessagesWaiting == pxQueue->uxLength )	{xReturn = ( ( BaseType_t ) 1 );}
		else                                            		{xReturn = ( ( BaseType_t ) 0 );}
	vPortExitCritical();
	return xReturn;
}
//*************************************************************************************************************************
BaseType_t xQueueIsQueueFullFromISR( const QueueHandle_t xQueue )
{ BaseType_t xReturn;
  Queue_t * const pxQueue = xQueue;
	if( pxQueue->uxMessagesWaiting == pxQueue->uxLength )	{xReturn = ( ( BaseType_t ) 1 ); }
	else	                                                {xReturn = ( ( BaseType_t ) 0 ); }
	return xReturn;
}  
