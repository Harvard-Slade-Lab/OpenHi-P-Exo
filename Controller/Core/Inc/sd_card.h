#ifndef SD_CARD_H_
#define SD_CARD_H_

#include <fatfs.h>

// SD Card Data Size
#define NODE_LEN 20
#define DATA_CNT 20
#define DATA_BLK_SIZE ((sizeof(TxData) + 4) * DATA_CNT)

// Structure declaration
typedef struct __node
{
    uint8_t data[DATA_BLK_SIZE];
    uint8_t count;
    struct __node *next;
} node;

// External variables
extern node buffer[NODE_LEN];
extern node *head;
extern uint8_t sd_buffer[DATA_BLK_SIZE];
extern UINT bc;

extern uint8_t sdState;
extern uint8_t logState;

// Function Prototypes
void init_node(void);
void write_data_to_sd(uint8_t *data);
uint8_t get_data_from_sd(uint8_t *data);
void sd_card_task(void);

#endif // SD_CARD_H_