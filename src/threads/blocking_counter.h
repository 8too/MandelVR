#pragma once

#include <mutex>
#include <condition_variable>

class blocking_counter
{
public:
	explicit blocking_counter() noexcept {}

	void increment() noexcept
	{
		{
			std::unique_lock<std::mutex> lock(m_mutex);
			m_counter++;
		}
	}

	void decrement() noexcept
	{
		bool trigger = false;
		{
            std::unique_lock<std::mutex> lock(m_mutex);
			m_counter--;
			if (m_counter <= 0) {
				trigger = true;
			}
		}
		if (trigger) {
			m_cv.notify_one();
		}
	}

	void wait() noexcept
	{
        std::unique_lock<std::mutex> lock(m_mutex);
		m_cv.wait(lock, [&]() { return m_counter <= 0; });
		m_counter = 0;
	}

	template<typename T>
	bool wait_for(T t) noexcept
	{
		std::unique_lock lock(m_mutex);
		bool result = m_cv.wait_for(lock, t, [&]() { return m_counter <= 0; });
		if (result) m_counter = 0;
		return result;
	}

	template<typename T>
	bool wait_until(T t) noexcept
	{
		std::unique_lock lock(m_mutex);
		bool result = m_cv.wait_until(lock, t, [&]() { return m_counter <= 0; });
		if (result) m_counter = 0;
		return result;
	}

private:
	int m_counter = 0;
	std::mutex m_mutex;
	std::condition_variable m_cv;
};