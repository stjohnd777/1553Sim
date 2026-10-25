//
// Created by Daniel St. John St. John on 10/21/26.
//

#include "mil1553/RT.hpp"
#include <algorithm>

RT::RT(uint8_t address) : address_(address) {}

uint8_t RT::getAddress() const {
    return address_;
}

StatusWord RT::handleTransmit(uint8_t /*subAddress*/, uint8_t /*wordCount*/, DataBlock1553& /*outData*/) {
    StatusWord status{address_};
    status.messageError = true; // Subaddress unhandled by default
    return status;
}

StatusWord RT::handleReceive(uint8_t /*subAddress*/, const DataBlock1553& /*inData*/) {
    StatusWord status{address_};
    status.messageError = true; // Subaddress unhandled by default
    return status;
}

void RT::stepSimulation(double /*dtSeconds*/) {
    // Default no-op
}

Hal1553::Transceiver& RT::getTransceiver() {
    return transceiver_;
}

const Hal1553::Transceiver& RT::getTransceiver() const {
    return transceiver_;
}

bool RT::executeHalReceive(Hal1553::Wire& wire, const CommandWord& cmd, const std::vector<Hal1553::Manchester::Waveform>& dataWaveforms, Hal1553::Manchester::Waveform& outStatusWave) {
    const uint8_t expectedWordCount = cmd.getEffectiveWordCount();

    // Validate expected waveform count and bounds
    if (dataWaveforms.size() != expectedWordCount || dataWaveforms.size() > DataBlock1553::MAX_WORDS) {
        StatusWord errStatus{address_};
        errStatus.messageError = true;
        transceiver_.tx.enable();
        outStatusWave = Hal1553::Manchester::encode(errStatus.toRaw(), WordType::Status);
        for (Level lvl : outStatusWave) {
            transceiver_.clock.tick();
            wire.drive(lvl);
        }
        wire.release();
        transceiver_.tx.disable();
        return false;
    }

    DataBlock1553 inData{};
    const size_t wordsToDecode = std::min<size_t>(dataWaveforms.size(), DataBlock1553::MAX_WORDS);
    inData.count = static_cast<uint8_t>(wordsToDecode);

    // Sample and decode each data waveform through RT transceiver RX
    for (size_t i = 0; i < wordsToDecode; ++i) {
        WordType wordType{};
        u16 rawVal = 0;
        if (!transceiver_.rx.enabled || !Hal1553::Manchester::decode(dataWaveforms[i], wordType, rawVal) || wordType != WordType::Data) {
            StatusWord errStatus{address_};
            errStatus.messageError = true;
            transceiver_.tx.enable();
            outStatusWave = Hal1553::Manchester::encode(errStatus.toRaw(), WordType::Status);
            for (Level lvl : outStatusWave) {
                transceiver_.clock.tick();
                wire.drive(lvl);
            }
            wire.release();
            transceiver_.tx.disable();
            return false;
        }
        inData.words[i] = rawVal;
    }

    StatusWord status = handleReceive(cmd.subAddress, inData);

    // Transmit StatusWord through RT transceiver TX onto physical wire
    transceiver_.tx.enable();
    outStatusWave = Hal1553::Manchester::encode(status.toRaw(), WordType::Status);
    for (Level lvl : outStatusWave) {
        transceiver_.clock.tick();
        wire.drive(lvl);
    }
    wire.release();
    transceiver_.tx.disable();

    return status.isOk();
}

bool RT::executeHalTransmit(Hal1553::Wire& wire, const CommandWord& cmd, Hal1553::Manchester::Waveform& outStatusWave, std::vector<Hal1553::Manchester::Waveform>& outDataWaves) {
    const uint8_t expectedWordCount = cmd.getEffectiveWordCount();
    DataBlock1553 outData{};
    StatusWord status = handleTransmit(cmd.subAddress, expectedWordCount, outData);

    // Validate that RT provided exactly the commanded word count
    if (status.isOk()) {
        if (outData.count != expectedWordCount || outData.count > DataBlock1553::MAX_WORDS || !outData.isValid()) {
            status.messageError = true;
        }
    }

    // Transmit StatusWord through RT transceiver TX onto physical wire
    transceiver_.tx.enable();
    outStatusWave = Hal1553::Manchester::encode(status.toRaw(), WordType::Status);
    for (Level lvl : outStatusWave) {
        transceiver_.clock.tick();
        wire.drive(lvl);
    }

    outDataWaves.clear();
    if (status.isOk()) {
        const uint8_t wordsToTransmit = std::min<uint8_t>(outData.count, DataBlock1553::MAX_WORDS);
        for (uint8_t i = 0; i < wordsToTransmit; ++i) {
            auto dataWave = Hal1553::Manchester::encode(outData.words[i], WordType::Data);
            for (Level lvl : dataWave) {
                transceiver_.clock.tick();
                wire.drive(lvl);
            }
            outDataWaves.push_back(dataWave);
        }
    }

    wire.release();
    transceiver_.tx.disable();

    return status.isOk();
}
