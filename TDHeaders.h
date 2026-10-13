#pragma once

// User-approved exceptions for the TD 2025.33230 vendor headers only.
// Keep warnings-as-errors enabled for every caller after this include.
#if defined(__clang__)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Winvalid-offsetof"
#pragma clang diagnostic ignored "-Wunused-parameter"
#pragma clang diagnostic ignored "-Wunused-private-field"
#pragma clang diagnostic ignored "-Wreorder-ctor"
#elif defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable: 4100 5038)
#endif

#include "CHOP_CPlusPlusBase.h"

#if defined(__clang__)
#pragma clang diagnostic pop
#elif defined(_MSC_VER)
#pragma warning(pop)
#endif
