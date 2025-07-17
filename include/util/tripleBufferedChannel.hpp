#pragma once

#include "util/channel.hpp"

#include <atomic>

// A single-slot channel where the last writer wins that is Triple Buffered to
// ensure the following guarantees,
//  - producer thread must not not be blocked in updating the buffer
//  - consumer thread must not be blocked in consuming the buffer
// Data loss is unavoidable...but that's kinda the point.
//
// Under this definition, we claim that the datastructure is linearisable.
//    Why?
// Our abstract model is a single slot where
//  - `send(value)` overwrites the current value
//      - The linearisation point is the atomic ready.exchange(...)
//      - This is the moment the new data becomes available to the consumer
//  - `receive()` reads the latest written value
//      - The linearisation point is its own ready.exchange(...)
//      - This is the moment the consumer acquires the new data for itself
// Then any execution history of this algorithm can be mapped to a valid
// sequential history of a single-slot buffer where writes can overwrite
// previous writes. (the full proof is an exercise to the reader)
template <typename T>
class TripleBufferedChannel
{
    std::atomic<T*> present;
    std::atomic<T*> ready;
    std::atomic<T*> inprogress;

    // previous buffer...tracks the previous buffer
    // for the simulation to be able to update state
    std::atomic<T*> previousWriteBuffer;
    std::atomic<bool> isNewDataAvailable;

    class ChannelSender : public Sender<T>
    {
        TripleBufferedChannel<T>& channel;

        void send() override
        {
            channel.send();
        }

        T& getWriteBuffer() override
        {
            return channel.getWriteBuffer();
        }

        T& getPreviousWriteBuffer() override
        {
            return channel.getPreviousWriteBuffer();
        }

      public:
        explicit ChannelSender(TripleBufferedChannel<T>& channel)
          : channel{channel}
        {
        }
    };

    class ChannelReceiver : public Receiver<T>
    {
        TripleBufferedChannel<T>& channel;

        const T& getBuffer() override
        {
            return channel.getBuffer();
        }

        bool isMessageReady() override
        {
            return channel.isNewDataAvailable.load();
        }

      public:
        explicit ChannelReceiver(TripleBufferedChannel<T>& channel)
          : channel{channel}
        {
        }
    };

    ChannelSender sender;
    ChannelReceiver receiver;

    void send()
    {
        T* writeBuf = inprogress.load();
        previousWriteBuffer.store(writeBuf);

        T* old_ready = ready.exchange(writeBuf);
        inprogress.store(old_ready);

        isNewDataAvailable.store(true);
    }

    T& getWriteBuffer()
    {
        return *inprogress.load();
    }

    T& getPreviousWriteBuffer()
    {
        T* ptr = previousWriteBuffer.load();
        if (ptr == nullptr) {
            ptr = inprogress.load();
        }
        return *ptr;
    }

    void consumed()
    {
    }

    const T& getBuffer()
    {
        bool expected = true;
        if (isNewDataAvailable.compare_exchange_strong(expected, false)) {
            T* old_ready = ready.exchange(present.load());
            present.store(old_ready);
        }
        // return old buffer if no new data, new one else
        return *present.load();
    }

  public:
    TripleBufferedChannel()
      : present(new T{}), ready(new T{}), inprogress(new T{}),
        previousWriteBuffer(nullptr), isNewDataAvailable(false), sender(*this),
        receiver(*this)
    {
    }

    TripleBufferedChannel(const TripleBufferedChannel&) = delete;
    TripleBufferedChannel(TripleBufferedChannel&&) = delete;
    TripleBufferedChannel& operator=(const TripleBufferedChannel&) = delete;
    TripleBufferedChannel& operator=(TripleBufferedChannel&&) = delete;

    ~TripleBufferedChannel()
    {
        delete present.load();
        delete ready.load();
        delete inprogress.load();
    }

    Sender<T>& getSender()
    {
        return sender;
    }

    Receiver<T>& getReceiver()
    {
        return receiver;
    }
};
