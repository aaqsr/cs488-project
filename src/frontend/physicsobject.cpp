#include "frontend/physicsobject.hpp"
#include "linalg.h"
#include <numbers>

namespace
{
linalg::aliases::float3 globalGravity = {0.0F, -9.8F, 0.0F};

} // namespace

/*********************************************************
INERTIA HELPER CLASS LOGIC
**********************************************************/
void Inertia::calcTemps(const linalg::aliases::float3& w, float& f1, float& f2, float& f3) {
    float tmp1 = w[0] + w[1];
    f1 = tmp1 + w[2];
    float tmp2 = w[0]*w[0];
    float tmp3 = tmp2 + w[1]*tmp1;
    f2 = tmp3 + w[2]*f1;
    f3 = w[0]*tmp2 + w[1]*tmp3 + w[2]*f2;
}
void Inertia::calcIntegrals(const linalg::aliases::float3& x, const linalg::aliases::float3& y, const linalg::aliases::float3& z) {
    // find surface integral using barycentric parametrisation
    // common temp values
    linalg::aliases::float3 f1, f2, f3;
    calcTemps(x, f1.x, f2.x, f3.x);
    calcTemps(y, f1.y, f2.y, f3.y);
    calcTemps(z, f1.z, f2.z, f3.z);
    #define g(w) f2.w+linalg::cmul(w, (w+f1.w))
    // Int{0, 1} (Int{0, 1-v} (x0+e1.x*u+e2.x*v) du) dv == (x0+x1+x2)/6
    int_x = f1.x/6;
    // Int{0, 1} (Int{0, 1-v} ((x0+e1.x*u+e2.x*v)^2) du) dv
    int_x2 = f2.x/12;
    // Int{0, 1} (Int{0, 1-v} ((y0+e1.y*u+e2.y*v)^2) du) dv
    int_y2 = f2.y/12;
    // Int{0, 1} (Int{0, 1-v} ((z0+e1.z*u+e2.z*v)^2) du) dv
    int_z2 = f2.z/12;
    int_x3 = f3.x/20;
    int_y3 = f3.y/20;
    int_z3 = f3.z/20;
    int_x2y = linalg::dot(y, g(x))/60;
    int_y2z = linalg::dot(z, g(y))/60;
    int_z2x = linalg::dot(x, g(z))/60;
}

void Inertia::add_face(const linalg::aliases::float3& v0, const linalg::aliases::float3& v1, const linalg::aliases::float3& v2, float density) {
    linalg::aliases::float3 edge1 = v1 - v0;
    linalg::aliases::float3 edge2 = v2 - v0;
    linalg::aliases::float3 norm = (linalg::cross(edge1, edge2));
    calcIntegrals({v0.x, v1.x, v2.x}, {v0.y, v1.y, v2.y}, {v0.z, v1.z, v2.z});
    float old_mass = mass;
    auto face_com = linalg::aliases::float3{int_x2*norm.x, int_y2*norm.y, int_z2*norm.z}/2;
    volume += norm.x * int_x;
    mass += norm.x * int_x * density;
    if (mass != 0.0f) com = (com * old_mass + density*face_com)/mass;
    inertia_xx += density * (int_y3*norm.y+int_z3*norm.z)/3;
    inertia_yy += density * (int_x3*norm.x+int_z3*norm.z)/3;
    inertia_zz += density * (int_x3*norm.x+int_y3*norm.y)/3;
    inertia_xy += density * int_x2y * norm.x / 2;
    inertia_yz += density * int_y2z * norm.y / 2;
    inertia_zx += density * int_z2x * norm.z / 2;
}
void Inertia::correct_inertia_with_com() {
    inertia_xx -= mass * (com.y * com.y + com.z * com.z);
    inertia_yy -= mass * (com.x * com.x + com.z * com.z);
    inertia_zz -= mass * (com.x * com.x + com.y * com.y);
    inertia_xy -= mass * com.x * com.y;
    inertia_yz -= mass * com.y * com.z;
    inertia_zx -= mass * com.z * com.x;
}

void Inertia::rescale(float scale) {
    // Faster rescaling when uniform
    float scaleFactor = static_cast<float>(linalg::pow(scale, 5));
    inertia_xx *= scaleFactor;
    inertia_yy *= scaleFactor;
    inertia_zz *= scaleFactor;
    inertia_xy *= scaleFactor;
    inertia_yz *= scaleFactor;
    inertia_zx *= scaleFactor;
    volume *= static_cast<float>(linalg::pow(scale, 3));
    mass *= static_cast<float>(linalg::pow(scale, 3));
    com *= scale;
}

linalg::aliases::float3x3 Inertia::initialInertiaMatrix() {
    linalg::aliases::float3x3 m;
    m[0] = {inertia_xx, -inertia_xy, -inertia_zx};
    m[1] = {-inertia_xy, inertia_yy, -inertia_yz};
    m[2] = {-inertia_zx, -inertia_yz, inertia_zz};
    return m;
}

/*********************************************************
RIGID BODY PHYSICS LOGIC
**********************************************************/

void PhysicsObj::calcInertia(linalg::aliases::float3 scale) {
    inertia = Inertia();
    for (const Mesh& mesh: model.getMeshes()) {
        for (size_t face_idx = 2; face_idx < mesh.getFaces().size(); face_idx += 3) {
            const auto v0 = mesh.getVertex(mesh.getFaces()[face_idx-2]);
            const auto v1 = mesh.getVertex(mesh.getFaces()[face_idx-1]);
            const auto v2 = mesh.getVertex(mesh.getFaces()[face_idx-0]);
            // process faces
            inertia.add_face(
                linalg::cmul(v0.position, scale), 
                linalg::cmul(v1.position, scale), 
                linalg::cmul(v2.position, scale), 
                density
            );
        }
    }
    inertia.correct_inertia_with_com(); // idk if to use this com or not
    com = inertia.com;
    mass = inertia.mass;
    initInvInertia = linalg::inverse(inertia.initialInertiaMatrix());

}

void PhysicsObj::rescale(const linalg::aliases::float3& scale) {
    // Expensive: O(faces)
    calcInertia(scale);
    model.scale = scale;
};
void PhysicsObj::rescale(const float& scale) {
    // Cheaper: O(1)
    inertia.rescale(scale);
    com = inertia.com;
    mass = inertia.mass;
    initInvInertia = linalg::inverse(inertia.initialInertiaMatrix());
    model.scale = linalg::aliases::float3{scale};
};

void PhysicsObj::update(float deltaTime) {
    linalg::aliases::float3 temp = position();
    if (impulse.x != 0 || impulse.y != 0 || impulse.z != 0) {
        // printf("WOOOOOO\n");
        prevPos -= impulse/mass * deltaTime;
        impulse = {0.0f, 0.0f, 0.0f};
    }
    position() = 2*position() - prevPos + deltaTime*deltaTime*(globalGravity + constantForces/mass);
    angularMomentum += torque * deltaTime;
    torque = {0.0f, 0.0f, 0.0f};
    auto R = model.rotation.toMatrix3x3();
    auto angularVelocity = mul(mul(mul(R, initInvInertia), linalg::transpose(R)), angularMomentum);
    float angularSpeed = linalg::length(angularVelocity);
    // For debugging motion:
    // printf("L = (%.3f, %.3f, %.3f), w = (%.3f, %.3f, %.3f), |w|*dt = %.3f\n", angularMomentum.x, angularMomentum.y, angularMomentum.z, angularVelocity.x, angularVelocity.y, angularVelocity.z, angularSpeed *deltaTime);
    // auto euler_angles = model.rotation.toEulerAngles();
    // printf("Roll = %.3f, Pitch = %.3f, Yaw = %.3f\n", euler_angles.x, euler_angles.y, euler_angles.z);
    if (angularSpeed > 1e-6F) {
        // Axis-Angle Method (easier to understand)
        // rotate(Quaternion::fromAxisAngle(linalg::normalize(angularVelocity), angularSpeed * deltaTime));
        // Derivative trick method (might be cheaper since no trig functions)
        model.rotation += Quaternion(linalg::aliases::float4{deltaTime*angularVelocity/2, 0.0f}) * model.rotation;
        model.rotation.normalize();
    }
    prevPos = temp;
}

void PhysicsObj::applyForce(linalg::aliases::float3 force, linalg::aliases::float3 contact) {
    auto dist = contact - com;
    impulse += force;
    torque += linalg::cross(dist, force);
}

// void PhysicsObj::addVelocity(linalg::aliases::float3 velocity) {
//     prevPos = position();
// 	position() += velocity * 0.02f;
// }

void PhysicsObj::setPosition(const linalg::aliases::float3& pos)
{
    position() = pos;
    prevPos = pos;
}

void PhysicsObj::move(const linalg::aliases::float3& displacement)
{
    prevPos = position();
    position() = position() + displacement;
}

void PhysicsObj::setOrientation(const Quaternion& quat)
{
    model.rotation = quat.normalized();
}

void PhysicsObj::rotate(const Quaternion& deltaRotation)
{
    model.rotation = (deltaRotation * model.rotation).normalized();
}

void PhysicsObj::rotateAroundAxis(const linalg::aliases::float3& axis,
                              float angleRadians)
{
    const Quaternion axisRotation =
      Quaternion::fromAxisAngle(axis, angleRadians);
    rotate(axisRotation);
}
