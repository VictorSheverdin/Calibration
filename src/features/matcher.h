#pragma once

#include "src/common/featureprocessor.h"

class Extractor
{
public:
    virtual ~Extractor() = default;

protected:
    Extractor() = default;

};

class Matcher
{
public:
    virtual ~Matcher() = default;

protected:
    Matcher() = default;

};
