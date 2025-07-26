#pragma once

#include "linalg.h"

// TODO: Do we want struct of arrays instead for CPU cache locality when doing
// computations on one attribute only?
struct Vertex
{
    // If this changes, go update setupVertexAttributes in mesh.cpp!!!
    // there is nothing we can do... :(
    // https://youtu.be/F0Gkr4MBEO0?si=yLd6Qd_I4eD8aRTK
    linalg::aliases::float3 position;
    linalg::aliases::float3 normal;
    linalg::aliases::float2 texCoords;

    bool operator==(const Vertex& other) const
    {
        return position == other.position && normal == other.normal &&
               texCoords == other.texCoords;
    }
};
