#include "message.h"
#include "msgHeaders.h"

void makeMessage(Message* message, uint8_t type, uint8_t id, uint8_t* data) {
	message->type = type;
	message->id = id;
	message->data = data;
}

void getMessage(Message* message, uint8_t* buffer) {
	uint8_t c = 0;
	buffer[c++] = START;
	buffer[c++] = message->type;
	buffer[c++] = message->id;
	switch(message->type) {
		case TEXT:
			for(int i = 0; message->data[i] != 0; ++i) {
				buffer[c++] = message->data[i];
			}
			break;
		case LIGHT_SENSOR:
		case SOUND_SENSOR:
		default:
			buffer[c++] = *(message->data);
			break;
	}
	buffer[c++] = END;
	buffer[c] = 0;
}
