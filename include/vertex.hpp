#pragma once

#include <linalg.h>

// TODO: Do we want struct of arrays instead for CPU cache locality when doing
// computations on one attribute only?
struct Vertex
{
    // If this changes, go update setupVertexAttributes below!
    linalg::aliases::float3 position;
    linalg::aliases::float4 colour;

    static void setupVertexAttributes();
};
