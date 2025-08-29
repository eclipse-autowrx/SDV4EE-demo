/**
 * Copyright (c) 2025 Bosch Global Software Technologies Private Limited.
 *
 * This program and the accompanying materials are made available under the
 * terms of the MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <pthread.h>

#include <array>
#include <cstring>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>


class mutex {
    pthread_mutex_t m_;

   public:
    using native_handle_type = pthread_mutex_t*;

    mutex() {
        pthread_mutexattr_t attr;

        int res = pthread_mutexattr_init(&attr);
        if (res != 0) {
            throw std::runtime_error{std::strerror(res)};
        }

        res = pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
        if (res != 0) {
            throw std::runtime_error{std::strerror(res)};
        }

        res = pthread_mutex_init(&m_, &attr);
        if (res != 0) {
            throw std::runtime_error{std::strerror(res)};
        }
    }

    ~mutex() { pthread_mutex_destroy(&m_); }

    mutex(const mutex&) = delete;
    mutex& operator=(const mutex&) = delete;

    void lock() {
        auto res = pthread_mutex_lock(&m_);
        if (res != 0) {
            throw std::runtime_error(std::strerror(res));
        }
    }

    void unlock() noexcept { pthread_mutex_unlock(&m_); }

    bool try_lock() noexcept { return pthread_mutex_trylock(&m_) == 0; }

    native_handle_type native_handle() noexcept { return &m_; };
};

