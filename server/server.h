#ifndef SERVER_H
#define	SERVER_H
typedef struct {
   uint32_t delta;
   uint32_t count;
}__attribute__((__packed__)) DataPacket;

typedef struct {
   uint8_t padding;
   float speed;    
}__attribute__((__packed__)) RefPacket;

#endif
