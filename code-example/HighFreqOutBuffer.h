#pragma once

#include <thread>
#include <memory>

#include "OutBuffer.h"

class HighFreqOutBuffer : public OutBuffer {
        /*
         *  Four parameters of high-frequency signals in the buffer config
         *  correspond to one value in the adapter config.
         *
         *  Example:
         *      buffers: {"name":"array", "conf":{"name":"High_1", "type":"uint32", "size":4}}
         *      adapter: {"name":"array", "conf":{"name":"High_1", "size":1}}
         */
        class Scheduler;

        uint8_t prevTimeFlagVal = 0;
        uint64_t hash;

        Iupplin* iupplin = nullptr;
        std::shared_ptr<Scheduler> sch;
        std::thread thread;

        int prepare(const std::vector<ValuesToChannelRule>& rulesVec);

        uint32_t murmurHash2A(const void* key, int len, uint32_t seed = 0);
public:
        HighFreqOutBuffer();

        int init(const rapidjson::Value& conf, State st) override;

        int flush() override;

        void reset() override;

        void loopEnd() override;
};
