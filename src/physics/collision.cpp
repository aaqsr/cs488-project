#include "physics/collision.hpp"
#include "linalg.h"
#include "physics/constants.hpp"
#include "physics/rigidBody.hpp"

namespace
{

linalg::aliases::float3
findValidPosition(const AABB& aabb, const RigidBodyData& rigidBody,
                  const linalg::aliases::float3& desiredPosition)
{

    const AABB& localAABB = rigidBody.getCharacteristics().getLocalAABB();
    linalg::aliases::float3x3 rotationMatrix =
      rigidBody.getOrientation().toMatrix3x3();

    AABB desiredAABB =
      localAABB.computeMovedAndScaledAABB(desiredPosition, rotationMatrix);

    // keep AABB in bounds :)
    linalg::aliases::float3 correction{0.0F, 0.0F, 0.0F};

    if (desiredAABB.min.x < aabb.min.x) {
        correction.x = aabb.min.x - desiredAABB.min.x;
    } else if (desiredAABB.max.x > aabb.max.x) {
        correction.x = aabb.max.x - desiredAABB.max.x;
    }

    if (desiredAABB.min.y < aabb.min.y) {
        correction.y = aabb.min.y - desiredAABB.min.y;
    } else if (desiredAABB.max.y > aabb.max.y) {
        correction.y = aabb.max.y - desiredAABB.max.y;
    }

    if (desiredAABB.min.z < aabb.min.z) {
        correction.z = aabb.min.z - desiredAABB.min.z;
    } else if (desiredAABB.max.z > aabb.max.z) {
        correction.z = aabb.max.z - desiredAABB.max.z;
    }

    return desiredPosition + correction;
}

} // namespace

// TODO: please oh god please refactor this please
void SuperCollider::keepWithinAABB(AABB aabb, RigidBodyData& rigidBody)
{
    linalg::aliases::float3 validPosition =
      findValidPosition(aabb, rigidBody, rigidBody.getWorldPosition());

    linalg::aliases::float3 correction =
      validPosition - rigidBody.getWorldPosition();
    float correctionMagnitude = linalg::length(correction);

    if (correctionMagnitude < 1e-6F) {
        return; // already in valid position
    }

    // position correction
    rigidBody.worldPosition = validPosition;

    // rotation constraint: if we're at a boundary, limit angular momentum to
    // prevent the object from rotating outside the bounds
    AABB currentAABB = rigidBody.computeAABB();

    // are we touching any boundary
    const float tolerance = 1e-4F;
    bool touchingBoundary =
      (std::abs(currentAABB.min.x - aabb.min.x) < tolerance ||
       std::abs(currentAABB.max.x - aabb.max.x) < tolerance) ||
      (std::abs(currentAABB.min.y - aabb.min.y) < tolerance ||
       std::abs(currentAABB.max.y - aabb.max.y) < tolerance) ||
      (std::abs(currentAABB.min.z - aabb.min.z) < tolerance ||
       std::abs(currentAABB.max.z - aabb.max.z) < tolerance);

    // TODO: Doesn't work well :(
    if (touchingBoundary) {
        // we project angular velocity to only allow rotations that keep the
        // object in bounds
        auto R = rigidBody.orientation.toMatrix3x3();
        auto bodyInvInertia = rigidBody.characteristics->getInitInvInertia();
        auto invInertiaWorldFrame =
          mul(mul(R, bodyInvInertia), linalg::transpose(R));
        auto currentAngularVel =
          mul(invInertiaWorldFrame, rigidBody.angularMomentum);

        // we test small rotations in each axis to see which ones cause
        // violations
        const float testAngle = 1e-5F;
        linalg::aliases::float3 projectedAngularVel = currentAngularVel;

        if (std::abs(currentAngularVel.x) > 1e-6F) {
            Quaternion testRotX = Quaternion::fromAxisAngle(
              linalg::aliases::float3{1.0F, 0.0F, 0.0F},
              testAngle * (currentAngularVel.x > 0 ? 1.0F : -1.0F));
            Quaternion testOrientation = rigidBody.orientation * testRotX;
            testOrientation.normalize();

            Quaternion originalOrientation = rigidBody.orientation;
            rigidBody.orientation = testOrientation;
            rigidBody.invalidateTriangleCache();

            linalg::aliases::float3 testValidPos =
              findValidPosition(aabb, rigidBody, rigidBody.getWorldPosition());
            bool wouldViolate =
              linalg::length(testValidPos - rigidBody.getWorldPosition()) >
              1e-6F;

            rigidBody.orientation = originalOrientation;
            rigidBody.invalidateTriangleCache();

            if (wouldViolate) {
                projectedAngularVel.x = 0.0F;
            }
        }
        if (std::abs(currentAngularVel.y) > 1e-6F) {
            Quaternion testRotY = Quaternion::fromAxisAngle(
              linalg::aliases::float3{0.0F, 1.0F, 0.0F},
              testAngle * (currentAngularVel.y > 0 ? 1.0F : -1.0F));
            Quaternion testOrientation = rigidBody.orientation * testRotY;
            testOrientation.normalize();

            Quaternion originalOrientation = rigidBody.orientation;
            rigidBody.orientation = testOrientation;
            rigidBody.invalidateTriangleCache();

            linalg::aliases::float3 testValidPos =
              findValidPosition(aabb, rigidBody, rigidBody.getWorldPosition());
            bool wouldViolate =
              linalg::length(testValidPos - rigidBody.getWorldPosition()) >
              1e-6F;

            rigidBody.orientation = originalOrientation;
            rigidBody.invalidateTriangleCache();

            if (wouldViolate) {
                projectedAngularVel.y = 0.0F;
            }
        }
        if (std::abs(currentAngularVel.z) > 1e-6F) {
            Quaternion testRotZ = Quaternion::fromAxisAngle(
              linalg::aliases::float3{0.0F, 0.0F, 1.0F},
              testAngle * (currentAngularVel.z > 0 ? 1.0F : -1.0F));
            Quaternion testOrientation = rigidBody.orientation * testRotZ;
            testOrientation.normalize();

            Quaternion originalOrientation = rigidBody.orientation;
            rigidBody.orientation = testOrientation;
            rigidBody.invalidateTriangleCache();

            linalg::aliases::float3 testValidPos =
              findValidPosition(aabb, rigidBody, rigidBody.getWorldPosition());
            bool wouldViolate =
              linalg::length(testValidPos - rigidBody.getWorldPosition()) >
              1e-6F;

            rigidBody.orientation = originalOrientation;
            rigidBody.invalidateTriangleCache();

            if (wouldViolate) {
                projectedAngularVel.z = 0.0F;
            }
        }

        // and now we can convert the corrected projected angular velocity back
        // to angular momentum
        auto inertiaWorldFrame = linalg::inverse(invInertiaWorldFrame);
        rigidBody.angularMomentum = mul(inertiaWorldFrame, projectedAngularVel);
    }

    rigidBody.invalidateTriangleCache();
}
SuperCollider::AABBCollisionInfo
SuperCollider::detectAABBCollision(const RigidBodyData& bodyA,
                                   const RigidBodyData& bodyB, size_t indexA,
                                   size_t indexB)
{
    AABBCollisionInfo collision;
    collision.bodyAIndex = indexA;
    collision.bodyBIndex = indexB;

    AABB aabbA = bodyA.computeAABB();
    AABB aabbB = bodyB.computeAABB();

    // check for overlap
    if (!aabbA.overlaps(aabbB)) {
        return collision; // hasCollision remains false
    }

    collision.hasCollision = true;

    // calculate penetration depths for each axis
    float overlapX =
      std::min(aabbA.max.x, aabbB.max.x) - std::max(aabbA.min.x, aabbB.min.x);
    float overlapY =
      std::min(aabbA.max.y, aabbB.max.y) - std::max(aabbA.min.y, aabbB.min.y);
    float overlapZ =
      std::min(aabbA.max.z, aabbB.max.z) - std::max(aabbA.min.z, aabbB.min.z);

    // find the axis of minimum penetration (this becomes our collision
    // normal)
    // TODO: should really be a function??
    if (overlapX <= overlapY && overlapX <= overlapZ) {
        collision.penetrationDepth = overlapX;
        collision.normal = {1.0F, 0.0F, 0.0F};

        // determine direction based on relative positions
        linalg::aliases::float3 centerA = (aabbA.max + aabbA.min) * 0.5F;
        linalg::aliases::float3 centerB = (aabbB.max + aabbB.min) * 0.5F;
        if (centerA.x < centerB.x) {
            collision.normal.x = -1.0F;
        }

        // contact point on X axis
        float contactX = (centerA.x > centerB.x) ? aabbA.min.x : aabbA.max.x;
        collision.contactPoint = {contactX,
                                  (std::max(aabbA.min.y, aabbB.min.y) +
                                   std::min(aabbA.max.y, aabbB.max.y)) *
                                    0.5F,
                                  (std::max(aabbA.min.z, aabbB.min.z) +
                                   std::min(aabbA.max.z, aabbB.max.z)) *
                                    0.5F};

    } else if (overlapY <= overlapZ) {
        collision.penetrationDepth = overlapY;
        collision.normal = {0.0F, 1.0F, 0.0F};

        linalg::aliases::float3 centerA = (aabbA.max + aabbA.min) * 0.5F;
        linalg::aliases::float3 centerB = (aabbB.max + aabbB.min) * 0.5F;
        if (centerA.y < centerB.y) {
            collision.normal.y = -1.0F;
        }

        // Contact point on Y axis
        float contactY = (centerA.y > centerB.y) ? aabbA.min.y : aabbA.max.y;
        collision.contactPoint = {(std::max(aabbA.min.x, aabbB.min.x) +
                                   std::min(aabbA.max.x, aabbB.max.x)) *
                                    0.5F,
                                  contactY,
                                  (std::max(aabbA.min.z, aabbB.min.z) +
                                   std::min(aabbA.max.z, aabbB.max.z)) *
                                    0.5F};

    } else {
        collision.penetrationDepth = overlapZ;
        collision.normal = {0.0F, 0.0F, 1.0F};

        linalg::aliases::float3 centerA = (aabbA.max + aabbA.min) * 0.5F;
        linalg::aliases::float3 centerB = (aabbB.max + aabbB.min) * 0.5F;
        if (centerA.z < centerB.z) {
            collision.normal.z = -1.0F;
        }

        // Contact point on Z axis
        float contactZ = (centerA.z > centerB.z) ? aabbA.min.z : aabbA.max.z;
        collision.contactPoint = {(std::max(aabbA.min.x, aabbB.min.x) +
                                   std::min(aabbA.max.x, aabbB.max.x)) *
                                    0.5F,
                                  (std::max(aabbA.min.y, aabbB.min.y) +
                                   std::min(aabbA.max.y, aabbB.max.y)) *
                                    0.5F,
                                  contactZ};
    }

    return collision;
}

void SuperCollider::resolveAABBCollision(RigidBodyData& bodyA,
                                         RigidBodyData& bodyB,
                                         const AABBCollisionInfo& collision)
{
    float MAGIC_VALUE = 50.0F;
    float massA = bodyA.getCharacteristics().getInertiaTensor().getMass();
    float massB = bodyB.getCharacteristics().getInertiaTensor().getMass();

    float totalMass = massA + massB;
    float ratioA = (massB / totalMass);
    float ratioB = (massA / totalMass);
    // new velecity ratios
    float ratio1 = (massA - massB) / totalMass;
    float ratio2A = 2 * massB / totalMass;
    float ratio2B = 2 * massA / totalMass;

    // we use half the overlap
    float correctionAmount = 0.5F * collision.penetrationDepth;
    linalg::aliases::float3 uA = bodyA.worldPosition - bodyA.prevWorldPosition;
    linalg::aliases::float3 uB = bodyB.worldPosition - bodyB.prevWorldPosition;
    linalg::aliases::float3 uA_along_dir = linalg::dot(collision.normal, uA) * collision.normal;
    linalg::aliases::float3 uB_along_dir = linalg::dot(-collision.normal, uB) * -collision.normal;
    linalg::aliases::float3 bodyA_F = -(
      + ratio1 * uA_along_dir // initial velocities remaining component along collision dir
      + ratio2A * -uB_along_dir
    ) * massA  / Physics::RigidBody::deltaT / Physics::RigidBody::deltaT / MAGIC_VALUE;
    linalg::aliases::float3 bodyB_F = -(
      - ratio1 * uB_along_dir // initial velocities remaining component along collision dir
      + ratio2B * -uA_along_dir
    ) * massB / Physics::RigidBody::deltaT / Physics::RigidBody::deltaT / MAGIC_VALUE;
    bodyA.accumulatedTorque += linalg::cross(collision.contactPoint - bodyA.getWorldPosOfCenterOfMass(), bodyA_F);
    bodyB.accumulatedTorque += linalg::cross(collision.contactPoint - bodyB.getWorldPosOfCenterOfMass(), bodyB_F);
    bodyA.worldPosition += correctionAmount * ratioA * collision.normal;
    bodyB.worldPosition -= correctionAmount * ratioB * collision.normal; 
}

void SuperCollider::processAABBCollisions(
  std::vector<RigidBodyData>& rigidBodies)
{
    if (rigidBodies.size() < 2) {
        return;
    }

    // (broad phase) : clear and populate spatial grid
    spatialGrid.clear();
    for (size_t i = 0; i < rigidBodies.size(); ++i) {
        if (rigidBodies[i].enabled) {
            AABB aabb = rigidBodies[i].computeAABB();
            spatialGrid.insert(i, aabb);
        }
    }

    auto potentialPairs = spatialGrid.getPotentialCollisions();

    // (narrow phase) : process only potential collisions
    for (const auto& pair : potentialPairs) {
        size_t i = pair.first;
        size_t j = pair.second;

        if (!rigidBodies[i].enabled || !rigidBodies[j].enabled) {
            continue;
        }

        AABBCollisionInfo collision =
          detectAABBCollision(rigidBodies[i], rigidBodies[j], i, j);

        if (collision.hasCollision) {
            resolveAABBCollision(rigidBodies[i], rigidBodies[j], collision);
        }
    }
}
