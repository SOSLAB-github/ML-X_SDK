/*
 * Copyright (c) 2023, SOSLAB, Inc. Team SSD.
 * All rights reserved.
 */

#ifndef SOS_FIFO_H
#define SOS_FIFO_H

#include "soslab_typedef.h"

/**
 *  @brief SOSLAB APIs [ for standard C++11 ]
 *  @author gnohead
 *  @date 2018. 11.
 */
namespace SOSLAB {
    /**
     *  @brief The thread safe Queue class
     */
    template <typename T>
    class Fifo
    {
    public:
        typedef void(*release_callback_t)(T& item);

        /**
         * @brief Constructor
         * @param max_size The maximum size of queue.
         */
        Fifo(std::size_t max_size) : MAX_SIZE_(max_size) {}
        ~Fifo() {}

        /**
         * @brief push to memory.
         * @param v the queue item
         */
        void push(const T& v) {
            std::lock_guard<std::mutex> lock(mutex_);
            if (que_.size() == MAX_SIZE_) {
                que_.pop_front();
            }
            que_.push_back(v);
        }

        /**
         * @brief pop from the memory.
         * @param v reference of the queue item.
         * @return false = queue is empty, true = queue is not empty
         */
        bool pop(T& v) {
            std::lock_guard<std::mutex> lock(mutex_);

            bool retval = !que_.empty();
            if (retval) {
                v = que_.front();
                que_.pop_front();
            }

            return retval;
        }

        /**
         * @brief size of queue
         * @return size of queue
         */
        std::size_t size() const {
            return que_.size();
        }

        /**
         * @brief maximum size of queue
         * @return maximum size of queue
         */
        std::size_t max_size() const {
            return MAX_SIZE_;
        }

        /**
         * @brief usage of queue
         * @return percent (integer)
         */
        int usage() const {
            return static_cast<int>((static_cast<float>(que_.size()) / static_cast<float>(MAX_SIZE_)) * 100);
        }

        /**
         * @brief check empty
         * @return true = queue is empty, false = queue is not empty.
         */
        bool empty() const {
            return que_.empty();
        }

        /**
         * @brief clear queue
         */
        void clear() {
            std::lock_guard<std::mutex> lock(mutex_);
            while (!que_.empty()) { que_.pop_front(); }
        }

    private:
        std::size_t MAX_SIZE_;
        std::deque<T> que_;
        std::mutex mutex_;
    };


}   /* namespace SOSLAB */

#endif // SOS_PLATFORM_HPP
