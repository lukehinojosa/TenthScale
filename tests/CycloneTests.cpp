#include <doctest/doctest.h>

#include <cyclone/cyclone.h>

TEST_CASE("A particle falls under its default gravity")
{
    cyclone::Particle particle;
    particle.setPosition(cyclone::Vector3(0, 10, 0));
    for (int i = 0; i < 50; i++)
    {
        particle.integrate((cyclone::real)0.02);
    }
    CHECK(particle.getPosition().y < 10);
    CHECK(particle.getVelocity().y < 0);
}

TEST_CASE("A rigid body with no forces keeps spinning about the same axis")
{
    cyclone::RigidBody body;
    body.setAcceleration(cyclone::Vector3(0, 0, 0));
    body.setAngularDamping(1);
    cyclone::Matrix3 inertia;
    inertia.setDiagonal(1, 1, 1);
    body.setInertiaTensor(inertia);
    body.setAngularVelocity(cyclone::Vector3(0, 1, 0));

    for (int i = 0; i < 10; i++)
    {
        body.integrate((cyclone::real)0.1);
    }
    CHECK(body.getPosition().magnitude() == doctest::Approx(0.0));
    CHECK(body.getAngularVelocity().y == doctest::Approx(1.0));
}
