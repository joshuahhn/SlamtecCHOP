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

#pragma once

#include "TDHeaders.h"
#include "Parameters.h"
#include "drvlogic/Receiver.h"

using namespace TD;

class SlamtecCHOP : public CHOP_CPlusPlusBase {
public:
    explicit SlamtecCHOP(const OP_NodeInfo*);
    ~SlamtecCHOP() override = default;
    void getGeneralInfo(CHOP_GeneralInfo*, const OP_Inputs*, void*) override;
    bool getOutputInfo(CHOP_OutputInfo*, const OP_Inputs*, void*) override;
    void getChannelName(int32_t, OP_String*, const OP_Inputs*, void*) override;
    void execute(CHOP_Output*, const OP_Inputs*, void*) override;
    void setupParameters(OP_ParameterManager*, void*) override;
    int32_t getNumInfoCHOPChans(void*) override;
    void getInfoCHOPChan(int32_t, OP_InfoCHOPChan*, void*) override;
    bool getInfoDATSize(OP_InfoDATSize*, void*) override;
    void getInfoDATEntries(int32_t, int32_t, OP_InfoDATEntries*, void*) override;
    void getWarningString(OP_String*, void*) override;
private:
    double ageMilliseconds() const noexcept;
    slamtec::Receiver receiver_;
    slamtec::Snapshot frame_;
    std::uint64_t executeCount_ = 0;
    bool active_ = false;
};
