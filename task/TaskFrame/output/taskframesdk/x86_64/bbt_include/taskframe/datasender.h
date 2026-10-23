#ifndef __DATASENDER_H__
#define __DATASENDER_H__

#include <cstdint>

#define DATA_SENDER_MAGIC 0x4453454E  // 'DSEN'

struct DataSenderData {
    uint32_t magic = DATA_SENDER_MAGIC;
    uint32_t version = 1;
    char reserved[64];
};

class CDataSender {
private:
    DataSenderData* data_;

public:
    CDataSender() : data_(nullptr) {}
    explicit CDataSender(DataSenderData* data) : data_(data) {}

    static void InitData(DataSenderData* data) {
        data->magic = DATA_SENDER_MAGIC;
        data->version = 1;
    }

    void writeData(int id, double data) {}
    void writeError(int id, const char* task, const char* msg) {}
};

#endif
