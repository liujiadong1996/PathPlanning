#pragma once

namespace common {
namespace math {

//! Used by Angle class to speed-up computation of trigonometric functions.
#define SIN_TABLE_SIZE 16385
extern const float SIN_TABLE[SIN_TABLE_SIZE];

}  // namespace math
}  // namespace common