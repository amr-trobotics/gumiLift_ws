#include <mutex>
#include <thread>

class RecursiveMutex 
{
    public:
        void lock() 
        {
            if (this_thread_id == std::this_thread::get_id()) 
            {
                ++count;
                return;
            }

            mtx.lock();
            this_thread_id = std::this_thread::get_id();
            count = 1;
        }

        void unlock() 
        {
            if (--count == 0 || this_thread_id != std::this_thread::get_id()) 
            {
                this_thread_id = std::thread::id();
                mtx.unlock();
            }
        }

    private:
        std::mutex mtx;
        std::thread::id this_thread_id;
        int count = 0;
};