#pragma once

#include <memory>
#include <mutex>

/**
 * @class Singleton
 * @brief A thread-safe, generic Singleton base class using the CRTP.
 * <blockquote>
 * The road to hell is paved with good intentions.
 * </blockquote>
 * @ingroup util
 * @tparam T The class to be made a singleton.
 *
 * @section Technicality
 * This class implements the Singleton design pattern using the **Curiously
 * Recurring Template Pattern (CRTP)**. A class `MyClass` that should be a
 * singleton must inherit from `Singleton<MyClass>`. This allows `Singleton` to
 * create an instance of `MyClass` without knowing its concrete type at compile
 * time.
 *
 * @section Performance and Thread Safety
 * Thread safety during initialization is guaranteed by `std::call_once` and
 * `std::once_flag`. The first time any thread calls `GetInstance()`, the lambda
 * function that creates the instance will be executed exactly once. All other
 * concurrent or subsequent calls will block until the initialization is
 * complete and then receive the same instance. This is the standard, modern C++
 * approach to creating thread-safe singletons and avoids the pitfalls of older
 * double-checked locking patterns.
 *
 * @section Data and Code Sources
 * Inspired by the singleton implementation provided by Refactoring Guru,
 * https://refactoring.guru/design-patterns/singleton/cpp/example although
 * theirs is not encapsulated in the CRTP pattern.
 *
 * @section Usage
 * To create a singleton class `MyManager`:
 * ```cpp
 * class MyManager : public Singleton<MyManager> {
 * friend class Singleton<MyManager>; // Grant access to private constructor
 * private:
 * MyManager() { // ... initialization ... }
 * public:
 * void doWork() { // ... }
 * };
 *
 * // To use:
 * MyManager::GetInstance().doWork();
 * ```
 * The constructor of the derived class must be made private or protected to
 * prevent manual instantiation, and `Singleton<T>` must be made a `friend` to
 * allow it to call the constructor.
 */
template <typename T>
class Singleton
{
    /**
     * @brief A unique pointer to the single instance of the class.
     */
    inline static std::unique_ptr<T> instance_ = nullptr;

    /**
     * @brief A flag to ensure the initialization routine is called exactly once
     * across all threads.
     */
    inline static std::once_flag initialized_;

  protected:
    /**
     * @brief Protected default constructor to allow inheritance.
     */
    Singleton() = default;

    /**
     * @brief Protected virtual destructor to allow for proper cleanup of
     * derived classes.
     */
    virtual ~Singleton() = default;

  public:
    // Delete copy/move constructors and assignment operators to enforce the
    // singleton property.
    Singleton(const Singleton&) = delete;
    Singleton(Singleton&&) = delete;
    Singleton& operator=(const Singleton&) = delete;
    Singleton& operator=(Singleton&&) = delete;

    /**
     * @brief Retrieves the single, global instance of the class.
     * @details If the instance does not yet exist, it will be created in a
     * thread-safe manner.
     * @return A reference to the singleton instance.
     */
    static T& GetInstance()
    {
        // Ensures that the instance is created only once, even if multiple
        // threads call this function concurrently for the first time.
        std::call_once(initialized_, [&]() {
            // The private constructor of T is called here.
            instance_ = std::unique_ptr<T>(new T);
        });
        return *instance_;
    }
};
