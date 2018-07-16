#ifndef SIMPLEMSG_H
#define SIMPLEMSG_H

namespace SimpleMsg {
enum  ID {
    ID_NONE,
    ID_SAMPLE,
    ID_CMD,
    ID_ACK,
    ID_REGRESULT,
    ID_BRDID,
    ID_CFG

};

enum  Commands {
    CMD_NOCMD,
    CMD_STARTCALIB,
    CMD_STOPCALIB ,
    CMD_SAVERESULT,
    CMD_RESETSTIFFNESS,
    CMD_STARTMOTORS,
    CMD_STOPMOTORS,
    CMD_GETID,
    CMD_GETRESULT,
    CMD_ZEROATI
};

typedef struct Msg_CFG{
    uint8_t id;
    float maxCurr;
    float maxTorque;
    float countsPerUnit;
} __attribute__ ( (packed) ) Msg_CFG;

typedef struct Msg_None {
    uint8_t id;
    uint8_t data[15];
} __attribute__ ( (packed) ) Msg_None;

typedef struct Msg_Cmd{
    uint8_t id;
    uint8_t command;
} __attribute__ ( (packed) ) Msg_Cmd;

typedef struct Msg_Bid{
    uint8_t id;
    int boardId;
} __attribute__ ( (packed) ) Msg_Bid;

typedef struct Msg_Ack{
    uint8_t id;
    uint8_t command;
} __attribute__ ( (packed) ) Msg_Ack;

typedef struct Msg_Curr{
    uint8_t id;
    double sample;
} __attribute__ ( (packed) ) Msg_Curr;

typedef struct Msg_Result{
    uint8_t id;
    float M;
    float B;
    float R;
} __attribute__ ( (packed) ) Msg_Result;

typedef union Msg {
    uint8_t id;
    Msg_None none;
    Msg_Cmd	cmd;
    Msg_Ack ack;
    Msg_Curr	curr;
    Msg_Result res;
    Msg_Bid bid;
    Msg_CFG cfg;
} __attribute__ ( (packed) ) Msg;

#define MSG_PROTOCOL_MESSAGE_LENGTH sizeof(Msg)

}

#endif // SIMPLEMSG_H
