#include <chrono>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <algorithm>
#include <string>
#include <cmath>
#include <unistd.h>

#include <boost/optional.hpp>

#include "HighFreqOutBuffer.h"
#include "BufferCommon.h"

class HighFreqOutBuffer::Scheduler {
        struct HighFreqParam {
                bool correctData = true; // To notify about wrong data at the start
                uint16_t type = 0;

                uint32_t period = 0;
                uint32_t dur = 0;
                uint32_t offset = 0;

                uint32_t prevPeriod = 0;
                uint32_t prevDur = 0;
                uint32_t prevOffset = 0;

                bool correct();
        };

        bool dataSet;
        bool timeUpdate;

        std::vector<bool> signalSent; // Information about whether the signal was output during the period
        std::vector<bool> prevType; // Bit array

        boost::optional<std::chrono::time_point<std::chrono::steady_clock>> startPoint;

        std::mutex dataMutex;
        std::condition_variable updateData;

        std::vector<std::pair<BaseAdapter*, unsigned>> adapters;
        std::vector<BaseAdapter*> adaptersForFlush;
        std::vector<HighFreqParam> channelsParam;
public:
        void updateTime() {
                timeUpdate = true;
                startPoint = std::chrono::steady_clock::now();
        }

        void run();

        friend class HighFreqOutBuffer;
};

bool HighFreqOutBuffer::Scheduler::HighFreqParam::correct() {
        if (offset >= period or offset + dur > period or !dur) {
                correctData = false;
        } else {
                correctData = true;
        }
        return correctData;
}

void HighFreqOutBuffer::Scheduler::run() {
        sched_param param;
        param.sched_priority = 95;
        sched_setscheduler(0, SCHED_FIFO, &param);

        dataSet = false;
        while (true) {
                std::unique_lock<std::mutex> m(dataMutex);
                if (dataSet) {
                        int64_t sleepTime = INT64_MAX;
                        auto curTime = std::chrono::steady_clock::now();
                        auto timeFromStart = std::chrono::duration_cast<std::chrono::microseconds> (curTime - startPoint.value()).count();

                        if (timeUpdate) {
                                size_t j = 0;
                                for (auto& i : channelsParam) {
                                        // when updating the time, we take into account the fictitious period up to this moment
                                        if (i.offset) {
                                                // to output the signal
                                                signalSent[j] = true;
                                        } else {
                                                signalSent[j] = false;
                                        }
                                        j++;
                                }
                                timeUpdate = false;
                        }

                        size_t j = 0;
                        for (auto& i : channelsParam) {
                                if (!i.correctData) {
                                        j++;
                                        continue;
                                }
                                auto n = timeFromStart / i.period;
                                auto firstPoint = n * i.period + i.offset;
                                auto secondPoint = n * i.period + i.offset + i.dur;
                                // wrtite data if time has come and signal was not sent before
                                if (firstPoint <= timeFromStart && timeFromStart <= secondPoint) {
                                        sleepTime = std::min(sleepTime, secondPoint);
                                        // signal
                                        if (!signalSent[j] or i.type != prevType[j]) {
                                                signalSent[j] = true;
                                                prevType[j] = i.type;
                                                adapters[j].first->writeData(adapters[j].second, static_cast<bool>(i.type));
                                        }
                                } else {
                                        auto tmp = firstPoint > timeFromStart ? firstPoint : (firstPoint + i.period);
                                        sleepTime = std::min(sleepTime, tmp);
                                        // background
                                        if (signalSent[j] or i.type != prevType[j]) {
                                                signalSent[j] = false;
                                                prevType[j] = i.type;
                                                adapters[j].first->writeData(adapters[j].second, static_cast<bool>(i.type ^ 1));
                                        }
                                }
                                j++;
                        }

                        for (auto& adapter : adaptersForFlush) {
                                adapter->flushOut();
                        }

                        // wait next point in the schedule
                        updateData.wait_until(m, *startPoint + std::chrono::microseconds(sleepTime));
                } else {
                        // wait for start experiment
                        updateData.wait(m);
                }

        }
}

HighFreqOutBuffer::HighFreqOutBuffer() : sch(std::make_shared<Scheduler>()), thread([this]() {this->sch->run();}) {
}

int HighFreqOutBuffer::prepare(const std::vector<ValuesToChannelRule>& rulesVec) {
        for (auto& i : buffsToValuesRules_) {
                if (i.name_ == "UpdateTime") {
                        continue;
                }
                auto it = customFind(rulesVec, i.name_ + "-000");
                if (it == rulesVec.end()) {
                        printf("param not found - %s\n", (i.name_ + "-000").c_str());
                        return -1;
                } else {
                        valuesToChannelRules_.push_back(*it);
                }
        }
        return 0;
}

int HighFreqOutBuffer::init(const rapidjson::Value& conf, State st) {
        iupplin = st.iupplin;
        if (!iupplin) {
                return -1;
        }

        buf_ = st.buf;
        if (buf_.type != IUPP_BUFF_TYPE::Input && buf_.type != IUPP_BUFF_TYPE::Async) {
                return -1;
        }

        auto res = parseRules1(conf);
        if (!res) {
                return -1;
        }

        buffsToValuesRules_ = std::move(*res);

        int r = rulesPrepare(buffsToValuesRules_, buf_.size);
        if (r < 0) {
                fprintf(stderr,
                        "ERROR[0x21]: buffer rulesPrepare. name - %s. invalid buffer size or "
                        "duplicate parameter names\n",
                        buf_.name.c_str());
                return -1;
        }

        if (prepare(*st.vec_rules) != 0) {
                return -1;
        }

        size_t j = 0;
        for (auto& i : buffsToValuesRules_) {
                if (i.name_ == "UpdateTime") {
                        continue;
                }
                sch->channelsParam.emplace_back();
                sch->signalSent.emplace_back();
                sch->prevType.emplace_back();
                sch->adapters.emplace_back(valuesToChannelRules_[j].adapter_, valuesToChannelRules_[j].channel);

                auto& tmp = sch->adaptersForFlush;
                if (std::find(tmp.begin(), tmp.end(), valuesToChannelRules_[j].adapter_) == tmp.end()) {
                        valuesToChannelRules_[j].adapter_->setType(AdapterType::ForHighFreqSignals);
                        tmp.emplace_back(valuesToChannelRules_[j].adapter_);
                }

                j++;
        }

        return 0;
}

int HighFreqOutBuffer::flush() {
        auto newHash = murmurHash2A(buf_.mem, buf_.size);
        if (hash == newHash) {
                return 0;
        }
        hash = newHash;

        std::unique_lock<std::mutex> m(sch->dataMutex);
        if (!sch->startPoint) {
                sch->updateTime();
        }
        sch->dataSet = true;
        size_t j = 0;
        for (auto& i : buffsToValuesRules_) {
                if (i.name_ == "UpdateTime") {
                        if (*(buf_.mem + i.begin_) && !prevTimeFlagVal) {
                                sch->updateTime();
                        }
                        prevTimeFlagVal = *(buf_.mem + i.begin_);
                        continue;
                }
                auto& channel = sch->channelsParam[j++];
                uint32_t* buf_begin = reinterpret_cast<uint32_t*>(buf_.mem + i.begin_);
                channel.prevPeriod = channel.period;
                channel.prevDur = channel.dur;
                channel.prevOffset = channel.offset;

                channel.period = buf_begin[0];
                channel.dur = buf_begin[1];
                channel.type = buf_begin[2];
                channel.offset = buf_begin[3];

                bool prevCorrectness = channel.correctData;
                if (!channel.correct() && (prevCorrectness or channel.prevPeriod != channel.period or channel.prevDur != channel.dur or channel.prevOffset != channel.offset)) {
                        std::string textMsg("Предупреждение: Неправильные параметры высокочастотной РК. Выдача значений высокочастотной РК отключена. Номер РК: ");
                        iupplin->sendMessage(textMsg + i.name_);
                }
        }
        sch->updateData.notify_one();

        return 0;
}

void HighFreqOutBuffer::reset() {
        hash = 0;
        std::memset(buf_.mem, 0, buf_.size);
}


void HighFreqOutBuffer::loopEnd() {
        std::unique_lock<std::mutex> m(sch->dataMutex);
        sch->startPoint.reset();
        sch->dataSet = false;
        size_t j = 0;
        for (auto& i : buffsToValuesRules_) {
                if (i.name_ == "UpdateTime") {
                        continue;
                }
                sch->channelsParam[j].correctData = true; // To notify about wrong data at the start
                sch->channelsParam[j].period = 0;
                sch->channelsParam[j].dur = 0;
                sch->channelsParam[j].type = 0;
                sch->channelsParam[j].offset = 0;

                sch->channelsParam[j].prevPeriod = 0;
                sch->channelsParam[j].prevDur = 0;
                sch->channelsParam[j++].prevOffset = 0;
        }
        sch->updateData.notify_one();
}

uint32_t HighFreqOutBuffer::murmurHash2A(const void* key, int len, uint32_t seed) {
        const uint32_t m = 0x5bd1e995;
        const int r = 24;
        uint32_t l = len;
        const unsigned char * data = (const unsigned char *) key;
        uint32_t h = seed;
        auto mmix = [&m, &r] (uint32_t& h, uint32_t& k) {k *= m;k ^= k >> r;k *= m;h *= m;h ^= k;};
        while (len >= 4) {
                uint32_t k = *(uint32_t*) data;
                mmix(h, k);
                data += 4;
                len -= 4;
        }

        uint32_t t = 0;
        switch (len) {
                case 3: t ^= data[2] << 16;
                case 2: t ^= data[1] << 8;
                case 1: t ^= data[0];
        }
        mmix(h, t);
        mmix(h, l);
        h ^= h >> 13;
        h *= m;
        h ^= h >> 15;
        return h;
}
