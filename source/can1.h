#ifndef CAN1_H_
#define CAN1_H_


unsigned char Can1_Configuration_mask(u8 FilterNumber, u16 ID, uint32_t id_type,  u16 ID_Mask , uint8_t sjw ,uint8_t bs1, uint8_t bs2, uint8_t prescale );

unsigned char Can1_Send_Message(CanTxMsg* TxMessage);

void Can1_Send_Ext(unsigned char id, unsigned char *data, int len, unsigned int id_type, unsigned int frame_type);

void Can1_Send(unsigned char id, unsigned char *data, int len);

int Can1_get( unsigned char *data, int size);

int Can1_getChar(unsigned char * data);



#endif 