#pragma once

#include "linalg.h"

// TODO: Do we want struct of arrays instead for CPU cache locality when doing
// computations on one attribute only?
struct Vertex
{
    // If this changes, go update setupVertexAttributes below!
    linalg::aliases::float3 position;
    linalg::aliases::float3 normal;
    linalg::aliases::float2 texCoords;

    static void setupVertexAttributes();

    bool operator==(const Vertex& other) const
    {
        return position == other.position && normal == other.normal &&
               texCoords == other.texCoords;
    }
};
