#pragma once

#include <atomic>

template <typename T>
class Sender
{
    friend class Message;
    virtual void send() = 0;
    virtual T& getWriteBuffer() = 0;
    virtual T& getPreviousWriteBuffer() = 0;

  public:
    class Message
    {
        Sender<T>& sender;

      public:
        Message(Sender<T>& sender) : sender{sender}
        {
        }
        Message(const Message&) = delete;
        Message(Message&&) = delete;
        Message& operator=(const Message&) = delete;
        Message& operator=(Message&&) = delete;

        ~Message()
        {
            sender.send();
        }

        T& getWriteBuffer()
        {
            return sender.getWriteBuffer();
        }

        T& getPreviousWriteBuffer()
        {
            return sender.getPreviousWriteBuffer();
        }
    };

    [[nodiscard]]
    Message createMessage()
    {
        return Message{*this};
    }

    virtual ~Sender() = default;
};

template <typename T>
class Receiver
{
    friend class Message;

    virtual const T& getBuffer() = 0;

  public:
    class Message
    {
        Receiver<T>& receiver;

      public:
        Message(Receiver<T>& receiver) : receiver{receiver}
        {
        }
        Message(const Message&) = delete;
        Message(Message&&) = delete;
        Message& operator=(const Message&) = delete;
        Message& operator=(Message&&) = delete;

        ~Message() = default;

        const T& getBuffer()
        {
            return receiver.getBuffer();
        }
    };

    [[nodiscard]]
    Message receive()
    {
        return Message{*this};
    }

    virtual ~Receiver() = default;

    virtual bool isMessageReady()
    {
        return true;
    }
};
