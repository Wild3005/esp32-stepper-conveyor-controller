#ifndef SHIFT_STEPPER_HPP
#define SHIFT_STEPPER_HPP

#include <AccelStepper.h>

class ShiftStepper : public AccelStepper {
public:
    ShiftStepper(uint8_t motorIndex,
                 void (*shiftOutFunc)(uint8_t))
        : AccelStepper(AccelStepper::FULL4WIRE, 0,0,0,0),
          _motorIndex(motorIndex),
          _shiftOut(shiftOutFunc) {}

    
    static uint8_t _buffer; // shared antar motor

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

    void setOutputPins(uint8_t mask) override {
    // bersihkan dulu area motor ini
    _buffer &= ~(0x0F << (_motorIndex * 4));

    // masukkan data baru
    uint8_t shifted = (mask & 0x0F) << (_motorIndex * 4);
    _buffer |= shifted;

    // //| LOG
    // Serial.print("M");
    // Serial.print(_motorIndex);
    // Serial.print(" MASK: ");
    // Serial.print(mask, BIN);
    // Serial.print(" BUFFER: ");
    // Serial.println(_buffer, BIN);

    _shiftOut(_buffer);
}

private:
    uint8_t _motorIndex;
    void (*_shiftOut)(uint8_t);

};

uint8_t ShiftStepper::_buffer = 0;

#endif