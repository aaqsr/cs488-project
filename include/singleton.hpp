#pragma once

#include <mutex>

// CRTP Singleton. Supposedly thread-safe...
template <typename T>
class Singleton
{
  public:
    // TODO: Be careful if threads call the fn with different args. Better to
    // just not allow args for now.
    //
    // template <typename... Args> static T& GetInstance(Args&&... args)
    static T& GetInstance()
    {
        std::call_once(initialized_, [&]() {
            // instance_ = std::make_unique<T>(std::forward<Args>(args)...);
            instance_ = std::make_unique<T>();
        });
        return *instance_;
    }

    Singleton(const Singleton&) = delete;
    Singleton(Singleton&&) = delete;
    Singleton& operator=(const Singleton&) = delete;
    Singleton& operator=(Singleton&&) = delete;

  protected:
    Singleton() = default;
    ~Singleton() = default;

  private:
    inline static std::unique_ptr<T> instance_ = nullptr;
    inline static std::once_flag initialized_;
};
