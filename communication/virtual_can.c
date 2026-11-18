
#include "can_interface.h"

static CAN_Message mailbox;
static bool message_available = false;

bool CAN_Transmit(const CAN_Message *message)
{
    if (message == NULL ||
        message->id > 0x7FFU ||
        message->dlc > CAN_MAX_DATA_LENGTH ||
        message_available) {
        return false;
    }

    mailbox = *message;
    message_available = true;

    return true;
}

bool CAN_Receive(CAN_Message *message)
{
    if (message == NULL || !message_available) {
        return false;
    }

    *message = mailbox;
    message_available = false;

    return true;
}
