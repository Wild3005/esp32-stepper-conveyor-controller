#ifndef SHIFT_STEPPER_HPP
#define SHIFT_STEPPER_HPP

#include <AccelStepper.h>
#include <ESP32Servo.h>

class ShiftStepper : public AccelStepper
{
public:
    ShiftStepper()
        : AccelStepper(AccelStepper::FULL4WIRE, 0, 0, 0, 0),
          _motorIndex(0),
          _shiftOut(nullptr)
    {
    }

    ShiftStepper(uint8_t motorIndex, void (*shiftOutFunc)(uint8_t *, size_t))
        : AccelStepper(AccelStepper::FULL4WIRE, 0, 0, 0, 0),
          _motorIndex(motorIndex),
          _shiftOut(shiftOutFunc) {}

    void init(uint8_t index,
              void (*callback)(uint8_t *, size_t))
    {
        _motorIndex = index;
        _shiftOut = callback;
    }

    static uint8_t *_buffer;   // shared antar motor
    static size_t _bufferSize; // ukuran buffer dalam byte

    static void begin(uint8_t *buffer, size_t size)
    {
        _buffer = buffer;
        _bufferSize = size;
    }

protected:
    void setOutputPins(uint8_t mask) override
    {

        int bitPos = _motorIndex * 4;

        // Validate buffer is initialized
        if (_buffer == nullptr || _bufferSize == 0)
        {
            return;
        }

        // Check if motor index will exceed buffer size
        int lastBitNeeded = bitPos + 3; // 4 bits per motor
        int lastByteNeeded = lastBitNeeded / 8;
        if (lastByteNeeded >= _bufferSize)
        {
            // Motor index exceeds buffer capacity
            return;
        }

        for (int i = 0; i < 4; i++)
        {

            int absoluteBit = bitPos + i;

            int byteIndex = absoluteBit / 8;

            int bitIndex = absoluteBit % 8;

            if (byteIndex >= _bufferSize)
                return;

            if (mask & (1 << i))
            {
                _buffer[byteIndex] |= (1 << bitIndex);
            }
            else
            {
                _buffer[byteIndex] &= ~(1 << bitIndex);
            }
        }

        if (_shiftOut != nullptr)
        {
            _shiftOut(_buffer, _bufferSize);
        }
    }

private:
    uint8_t _motorIndex;
    void (*_shiftOut)(uint8_t *, size_t);
};

// uint32_t ShiftStepper::_buffer = 0;
uint8_t *ShiftStepper::_buffer = nullptr;
size_t ShiftStepper::_bufferSize = 0;

#endif