// license:BSD-3-Clause
// copyright-holders:Hannes Janetzek
//============================================================
//
//  spscqueue.h - lock-free single-producer single-consumer
//  queue used to pass audio and MIDI between the host audio
//  thread and the emulation thread
//
//============================================================
#ifndef MAME_OSD_PLUGIN_SPSCQUEUE_H
#define MAME_OSD_PLUGIN_SPSCQUEUE_H

#pragma once

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <memory>


template <typename T>
class spsc_queue
{
public:
	// capacity is rounded up to a power of two
	explicit spsc_queue(std::size_t capacity)
	{
		std::size_t size = 1;
		while (size < capacity)
			size <<= 1;
		m_mask = size - 1;
		m_data = std::make_unique<T []>(size);
	}

	std::size_t capacity() const noexcept { return m_mask + 1; }

	// number of elements available to the consumer
	std::size_t size() const noexcept
	{
		return m_head.load(std::memory_order_acquire) - m_tail.load(std::memory_order_acquire);
	}

	// producer side: append up to count elements, returns the number appended
	std::size_t push(T const *data, std::size_t count) noexcept
	{
		std::size_t const head = m_head.load(std::memory_order_relaxed);
		std::size_t const tail = m_tail.load(std::memory_order_acquire);
		count = std::min(count, capacity() - (head - tail));
		for (std::size_t i = 0; i < count; i++)
			m_data[(head + i) & m_mask] = data[i];
		m_head.store(head + count, std::memory_order_release);
		return count;
	}

	bool push(T const &value) noexcept { return push(&value, 1) == 1; }

	// consumer side: look at the oldest element without removing it
	T const *front() const noexcept
	{
		std::size_t const tail = m_tail.load(std::memory_order_relaxed);
		if (m_head.load(std::memory_order_acquire) == tail)
			return nullptr;
		return &m_data[tail & m_mask];
	}

	// consumer side: remove up to count elements, returns the number removed;
	// data may be nullptr to discard elements
	std::size_t pop(T *data, std::size_t count) noexcept
	{
		std::size_t const tail = m_tail.load(std::memory_order_relaxed);
		std::size_t const head = m_head.load(std::memory_order_acquire);
		count = std::min(count, head - tail);
		if (data)
		{
			for (std::size_t i = 0; i < count; i++)
				data[i] = m_data[(tail + i) & m_mask];
		}
		m_tail.store(tail + count, std::memory_order_release);
		return count;
	}

private:
	std::unique_ptr<T []> m_data;
	std::size_t m_mask;
	alignas(64) std::atomic<std::size_t> m_head{ 0 };
	alignas(64) std::atomic<std::size_t> m_tail{ 0 };
};

#endif // MAME_OSD_PLUGIN_SPSCQUEUE_H
