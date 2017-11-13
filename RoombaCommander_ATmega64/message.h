#include <stdint.h>

#ifndef MESSAGE_H_
#define MESSAGE_H_

typedef struct _Message {
	uint8_t type;
	uint8_t id;
	uint8_t* data;
} Message;

void makeMessage(Message* message, uint8_t type, uint8_t id, uint8_t* data);
void getMessage(Message* message, uint8_t* buffer);

#endif /* MESSAGE_H_ */