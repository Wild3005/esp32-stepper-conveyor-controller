#ifndef SHIFT_STEPPER_HPP
#define SHIFT_STEPPER_HPP

#include <AccelStepper.h>
#include <ESP32Servo.h>

class ShiftStepper : public AccelStepper {
public:
    ShiftStepper()
        : AccelStepper(AccelStepper::FULL4WIRE, 0,0,0,0),
          _motorIndex(0),
          _shiftOut(nullptr)
    {}

    ShiftStepper(uint8_t motorIndex, void (*shiftOutFunc)(uint8_t*,size_t))
        : AccelStepper(AccelStepper::FULL4WIRE, 0,0,0,0),
          _motorIndex(motorIndex),
          _shiftOut(shiftOutFunc) {}

    void init(uint8_t index,
              void (*callback)(uint8_t*, size_t)){
        _motorIndex = index;
        _shiftOut = callback;
    }

    
    static uint8_t* _buffer; // shared antar motor
    static size_t _bufferSize; // ukuran buffer dalam byte

    static void begin(uint8_t* buffer, size_t size){
        _buffer = buffer;
        _bufferSize = size;
    }

protected:
    // void setOutputPins(uint8_t mask) override {
    //     // ambil posisi 4 bit sesuai motor
    //     uint8_t shifted = mask << (_motorIndex * 4);

    //     // simpan ke buffer global
    //     _buffer = (_buffer & ~(0x0F << (_motorIndex * 4))) | shifted;

    //     // kirim ke shift register
    //     Serial.print("BUFFER: ");
    //     Serial.println(_buffer, BIN);

    //     _shiftOut(_buffer);
    //     _shiftOut(_buffer);
    // }

    // void setOutputPins(uint8_t mask) override {
    //     // bersihkan dulu area motor ini
    //     _buffer &= ~(static_cast<uint32_t>(0x0F) << (_motorIndex * 4));

    //     // masukkan data baru
    //     uint32_t shifted = (static_cast<uint32_t>(mask) & 0x0F) << (_motorIndex * 4);
    //     _buffer |= shifted;

    //     // //| LOG
    //     // Serial.print("M");
    //     // Serial.print(_motorIndex);
    //     // Serial.print(" MASK: ");
    //     // Serial.print(mask, BIN);
    //     // Serial.print(" BUFFER: ");
    //     // Serial.println(_buffer, BIN);

    //     _shiftOut(_buffer);
    // }

    // void setOutputPins(uint8_t mask) override {

    //     int bitPos = _motorIndex * 4;

    //     int byteIndex = bitPos / 8;
    //     int bitOffset = bitPos % 8;

    //     if(byteIndex >= _bufferSize) return;

    //     _buffer[byteIndex] &= ~(0x0F << bitOffset);

    //     _buffer[byteIndex] |=
    //         static_cast<uint8_t>((mask & 0x0F) << bitOffset);

    //     _shiftOut(_buffer, _bufferSize);
    // }

    void setOutputPins(uint8_t mask) override {

        int bitPos = _motorIndex * 4;

        // Validate buffer is initialized
        if(_buffer == nullptr || _bufferSize == 0) {
            return;
        }

        // Check if motor index will exceed buffer size
        int lastBitNeeded = bitPos + 3; // 4 bits per motor
        int lastByteNeeded = lastBitNeeded / 8;
        if(lastByteNeeded >= _bufferSize) {
            // Motor index exceeds buffer capacity
            return;
        }

        for(int i = 0; i < 4; i++) {

            int absoluteBit = bitPos + i;

            int byteIndex = absoluteBit / 8;

            int bitIndex = absoluteBit % 8;

            if(byteIndex >= _bufferSize) return;

            if(mask & (1 << i)) {
                _buffer[byteIndex] |= (1 << bitIndex);
            } else {
                _buffer[byteIndex] &= ~(1 << bitIndex);
            }
        }

        if(_shiftOut != nullptr) {
            _shiftOut(_buffer, _bufferSize);
        }
    }




private:
    uint8_t _motorIndex;
    void (*_shiftOut)(uint8_t*, size_t);

};

// uint32_t ShiftStepper::_buffer = 0;
uint8_t* ShiftStepper::_buffer = nullptr;
size_t ShiftStepper::_bufferSize = 0;

#endif