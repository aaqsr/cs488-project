#pragma once

#include <memory>
#include <mutex>

// CRTP Singleton. Supposedly thread-safe...
template <typename T>
class Singleton
{
    inline static std::unique_ptr<T> instance_ = nullptr;
    inline static std::once_flag initialized_;

  protected:
    Singleton() = default;
    virtual ~Singleton() = default;

  public:
    Singleton(const Singleton&) = delete;
    Singleton(Singleton&&) = delete;
    Singleton& operator=(const Singleton&) = delete;
    Singleton& operator=(Singleton&&) = delete;

    // TODO: Be careful if threads call the fn with different args. Better to
    // just not allow args for now.
    //
    // template <typename... Args> static T& GetInstance(Args&&... args)
    static T& GetInstance()
    {
        std::call_once(initialized_, [&]() {
            // instance_ = std::make_unique<T>(std::forward<Args>(args)...);
            instance_ = std::unique_ptr<T>(new T);
        });
        return *instance_;
    }
};
