/* Shared Use License: This file is owned by Derivative Inc. (Derivative)
* and can only be used, and/or modified for use, in conjunction with
* Derivative's TouchDesigner software, and only if you are a licensee who has
* accepted Derivative's TouchDesigner license or assignment agreement
* (which also govern the use of this file). You may share or redistribute
* a modified version of this file provided the following conditions are met:
*
* 1. The shared file or redistribution must retain the information set out
* above and this list of conditions.
* 2. Derivative's name (Derivative Inc.) or its trademarks may not be used
* to endorse or promote products derived from this file without specific
* prior written permission from Derivative.
*/

#include "SlamtecCHOP.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <string>

extern "C" {
DLLEXPORT void FillCHOPPluginInfo(CHOP_PluginInfo* info) {
    if (!info->setAPIVersion(CHOPCPlusPlusAPIVersion)) return;
    auto& custom = info->customOPInfo;
    custom.opType->setString("Slamtec");
    custom.opLabel->setString("SlamtecLidar");
    custom.opIcon->setString("SLT");
    custom.authorName->setString("Vasily Betin");
    custom.authorEmail->setString("b@vasily.onl");
    custom.minInputs = 0;
    custom.maxInputs = 0;
}
DLLEXPORT CHOP_CPlusPlusBase* CreateCHOPInstance(const OP_NodeInfo* info) {
    try { return new SlamtecCHOP(info); } catch (...) { return nullptr; }
}
DLLEXPORT void DestroyCHOPInstance(CHOP_CPlusPlusBase* instance) { delete static_cast<SlamtecCHOP*>(instance); }
}

SlamtecCHOP::SlamtecCHOP(const OP_NodeInfo*)
    : receiver_([] { return slamtec::makeSdkBackend(); }) {}

void SlamtecCHOP::getGeneralInfo(CHOP_GeneralInfo* info, const OP_Inputs*, void*) {
    info->cookEveryFrameIfAsked = true;
    info->timeslice = false;
    info->inputMatchIndex = 0;
}

bool SlamtecCHOP::getOutputInfo(CHOP_OutputInfo* info, const OP_Inputs* inputs, void*) {
    info->numChannels = 4;
    info->numSamples = 360 * std::clamp(Parameters::evalPrecision(inputs), 1, 4);
    info->startIndex = 0;
    info->sampleRate = 60;
    return true;
}

void SlamtecCHOP::getChannelName(int32_t index, OP_String* name, const OP_Inputs* inputs, void*) {
    const bool polar = Parameters::evalCoord(inputs) == CoordMenuItems::Polar;
    const std::array<const char*, 4> names{polar ? "angle" : "x", polar ? "distance" : "y", "quality", "flag"};
    name->setString(index >= 0 && index < 4 ? names[static_cast<std::size_t>(index)] : "");
}

void SlamtecCHOP::execute(CHOP_Output* output, const OP_Inputs* inputs, void*) {
    ++executeCount_;
    active_ = Parameters::evalActive(inputs) == 1;
    // Initialize the actual TD allocation, even if dimensions changed this cook.
    for (int32_t c = 0; c < output->numChannels; ++c)
        std::fill_n(output->channels[c], output->numSamples, -1.0f);
    try {
        slamtec::Request request;
        request.active = active_;
        const bool network = Parameters::evalConnectionType(inputs) == 1;
        request.connection.transport = network ? (Parameters::evalNetworkType(inputs) == 1 ?
            slamtec::Transport::Udp : slamtec::Transport::Tcp) : slamtec::Transport::Serial;
        const char* address = inputs->getParString(network ? IpName : PortName);
        request.connection.address = address ? address : "";
        request.connection.baudOrPort = network ? inputs->getParInt(NetworkPortName) :
            slamtec::baudFromMenu(inputs->getParInt(BaudrateName));
        request.connection.standardMode = Parameters::evalStandart(inputs) == 1;
        request.output.precision = std::clamp(Parameters::evalPrecision(inputs), 1, 4);
        request.output.qualityCheck = Parameters::evalQuality(inputs) == 1;
        double minimum = 0, maximum = 40;
        inputs->getParDouble2(DistanceName, minimum, maximum);
        request.output.minimumMm = static_cast<float>(minimum * 1000);
        request.output.maximumMm = static_cast<float>(maximum * 1000);
        // Both control and data handoff are non-waiting. A busy control mailbox
        // is retried on the next cook; stop clears output immediately.
        receiver_.request(request);
        receiver_.consume(frame_);
        for (const char* parameter : {StandartModeName, ConnectName, PortName, BaudrateName,
                                     IpName, NetworkPortName, NetworkTypeName, PrecisionName})
            inputs->enablePar(parameter, !active_);
        inputs->enablePar(IpName, !active_ && network);
        inputs->enablePar(NetworkPortName, !active_ && network);
        inputs->enablePar(NetworkTypeName, !active_ && network);
        inputs->enablePar(PortName, !active_ && !network);
        inputs->enablePar(BaudrateName, !active_ && !network);
        if (!active_ || frame_.state != slamtec::State::Streaming ||
            frame_.generation != receiver_.generation() || ageMilliseconds() > 500 ||
            frame_.sampleCount != static_cast<std::size_t>(360 * request.output.precision) ||
            output->numChannels < 4) return;
        constexpr float radians = 3.14159265358979323846f / 180;
        const bool polar = Parameters::evalCoord(inputs) == CoordMenuItems::Polar;
        const auto count = std::min(frame_.sampleCount, static_cast<std::size_t>(output->numSamples));
        for (std::size_t i = 0; i < count; ++i) {
            const auto& sample = frame_.samples[i];
            output->channels[0][i] = polar ? sample.angle : sample.distance * std::cos(sample.angle * radians);
            output->channels[1][i] = polar ? sample.distance : sample.distance * std::sin(sample.angle * radians);
            output->channels[2][i] = sample.quality;
            output->channels[3][i] = sample.flag;
        }
    } catch (const std::exception& exception) {
        frame_.message = exception.what();
        frame_.state = slamtec::State::Backoff;
    } catch (...) {
        frame_.message = "Receiver update failed";
        frame_.state = slamtec::State::Backoff;
    }
}

void SlamtecCHOP::setupParameters(OP_ParameterManager* manager, void*) { Parameters::setup(manager); }

double SlamtecCHOP::ageMilliseconds() const noexcept {
    if (frame_.captured == std::chrono::steady_clock::time_point{}) return -1;
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - frame_.captured).count();
}

int32_t SlamtecCHOP::getNumInfoCHOPChans(void*) { return 11; }
void SlamtecCHOP::getInfoCHOPChan(int32_t index, OP_InfoCHOPChan* channel, void*) {
    const auto stats = receiver_.statistics();
    const std::array<const char*, 11> names{"executeCount", "received Samples", "Random", "scanSequence",
        "scanAgeMs", "droppedSnapshots", "receiveTimeouts", "receiveErrors", "reconnects", "capacityHits", "invalidNodes"};
    const std::array<double, 11> values{static_cast<double>(executeCount_), static_cast<double>(frame_.nodeCount), 0,
        static_cast<double>(frame_.sequence), ageMilliseconds(), static_cast<double>(stats.dropped),
        static_cast<double>(stats.timeouts), static_cast<double>(stats.errors), static_cast<double>(stats.reconnects),
        static_cast<double>(stats.capacityHits), static_cast<double>(stats.invalidNodes)};
    if (index >= 0 && index < static_cast<int32_t>(names.size())) {
        channel->name->setString(names[static_cast<std::size_t>(index)]);
        channel->value = static_cast<float>(values[static_cast<std::size_t>(index)]);
    }
}
bool SlamtecCHOP::getInfoDATSize(OP_InfoDATSize* size, void*) {
    size->rows = 14; size->cols = 2; size->byColumn = false; return true;
}
void SlamtecCHOP::getInfoDATEntries(int32_t index, int32_t entriesCount, OP_InfoDATEntries* entries, void*) {
    const std::array<const char*, 14> keys{"executeCount", "Lidar status", "Serial number", "Firmware version",
        "Hardware version", "Model", "Motor max/min/desired speed", "Last reading", "Available scan modes",
        "Current scan mode", "Motor control", "SDK source", "TouchDesigner target", "Output units"};
    const std::array<std::string, 14> values{std::to_string(executeCount_), frame_.message.c_str(), frame_.device.serial.c_str(),
        frame_.device.firmware.c_str(), frame_.device.hardware.c_str(), frame_.device.model.c_str(), "Not queried",
        std::to_string(frame_.nodeCount), frame_.device.scanModes.c_str(), frame_.device.scanMode.c_str(), frame_.device.motorControl.c_str(),
        "99478e5 + tracked safety patch", "2025.33230", "degrees / millimetres"};
    if (entriesCount >= 2 && index >= 0 && index < static_cast<int32_t>(keys.size())) {
        entries->values[0]->setString(keys[static_cast<std::size_t>(index)]);
        entries->values[1]->setString(values[static_cast<std::size_t>(index)].c_str());
    }
}
void SlamtecCHOP::getWarningString(OP_String* warning, void*) {
    if (active_ && (frame_.state != slamtec::State::Streaming || ageMilliseconds() > 500))
        warning->setString(frame_.state == slamtec::State::Streaming ? "Scan data is stale" : frame_.message.c_str());
}
