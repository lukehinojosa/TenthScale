#include "SandboxGame.h"
#include "GameEvents.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <utility>

#include <raymath.h>
#include <rlgl.h>
#include <imgui.h>

#include <core/EventSystem.h>
#include <core/MemoryTracker.h>
#include <engine/Conversions.h>

#ifdef TEMPLATE_HAS_SOCKETS
#include <net/Serialize.h>
#endif

#ifdef TEMPLATE_HAS_GPU_COMPUTE
#include <chrono>
#include <cmath>
#include <random>

#include <engine/ShaderUtils.h>
#endif

namespace
{
    constexpr float CUBE_HALF_SIZE = 1.0f;
    constexpr float FLOCK_AREA = 45.0f; // Boids stay within this distance of the origin on x and z
    constexpr float FLOCK_ALTITUDE = 5.0f; // Height above the terrain
    constexpr float PLATFORM_HALF_SIZE = 7.0f;
    constexpr int MAX_THROWN_BALLS = 40;
    constexpr int MAX_PLAYGROUND_BODIES = 220;
    constexpr float FRACTURE_SPEED = 3.0f; // A change of velocity bigger than this in one step breaks a fragile body

    Mesh buildConvexMesh(const cyclone::ConvexCollider& shape)
    {
        std::vector<float> positions;
        std::vector<float> normals;
        const auto& vertices = shape.getVertices();
        for (const auto& face : shape.getFaces())
        {
            for (std::size_t k = 1; k + 1 < face.indices.size(); k++)
            {
                for (int corner : { face.indices[0], face.indices[k], face.indices[k + 1] })
                {
                    positions.push_back((float)vertices[corner].x);
                    positions.push_back((float)vertices[corner].y);
                    positions.push_back((float)vertices[corner].z);
                    normals.push_back((float)face.normal.x);
                    normals.push_back((float)face.normal.y);
                    normals.push_back((float)face.normal.z);
                }
            }
        }
        Mesh mesh{};
        mesh.vertexCount = (int)positions.size() / 3;
        mesh.triangleCount = mesh.vertexCount / 3;
        mesh.vertices = (float*)MemAlloc((unsigned int)(positions.size() * sizeof(float)));
        mesh.normals = (float*)MemAlloc((unsigned int)(normals.size() * sizeof(float)));
        std::copy(positions.begin(), positions.end(), mesh.vertices);
        std::copy(normals.begin(), normals.end(), mesh.normals);
        UploadMesh(&mesh, false);
        return mesh;
    }

#ifdef TEMPLATE_HAS_GPU_COMPUTE
    constexpr Vector3 BALL_BOX_CENTER = { -26.0f, 16.0f, -8.0f };
    constexpr float BALL_BOX_HALF_SIZE = 8.0f;

    // One sphere mesh drawn once per ball through instancing, coloured by height
    const char* const BALL_VERTEX_SHADER = R"(
in vec3 vertexPosition;
in vec3 vertexNormal;
in mat4 instanceTransform;
uniform mat4 mvp;
out vec3 fragNormal;
out float fragHeight;
void main()
{
    vec4 world = instanceTransform * vec4(vertexPosition, 1.0);
    fragNormal = mat3(instanceTransform) * vertexNormal;
    fragHeight = world.y;
    gl_Position = mvp * world;
}
)";

    const char* const BALL_FRAGMENT_SHADER = R"(
in vec3 fragNormal;
in float fragHeight;
uniform vec3 sunDirection;
uniform vec2 heightRange;
out vec4 finalColor;
void main()
{
    float t = clamp((fragHeight - heightRange.x) / (heightRange.y - heightRange.x), 0.0, 1.0);
    vec3 base = mix(vec3(0.15, 0.45, 0.95), vec3(1.0, 0.55, 0.15), t);
    float light = 0.3 + 0.7 * max(dot(normalize(fragNormal), normalize(sunDirection)), 0.0);
    finalColor = vec4(base * light, 1.0);
}
)";
#endif

#ifdef TEMPLATE_HAS_SOCKETS
    constexpr net::MessageType PING = 1;
    constexpr net::MessageType PONG = 2;

    struct PingMessage
    {
        std::uint32_t sequence = 0;
        float sentAt = 0.0f;
    };

    template<net::IsStream Stream>
    bool serializePing(Stream& stream, PingMessage& message)
    {
        bool ok = net::serializeInt(stream, message.sequence, 0, 0xFFFFFFFFll);
        ok = ok && net::serializeRawFloat(stream, message.sentAt);
        return ok;
    }
#endif
}

SandboxGame::SandboxGame(const SandboxSettings& settings) : Game(settings.game), mRestitution(settings.restitution)
{
#ifdef TEMPLATE_HAS_RELAY
    std::snprintf(mRelayUrl, sizeof(mRelayUrl), "%s", settings.relayUrl.c_str());
#endif
}

void SandboxGame::onInit()
{
    mpTranslator = std::make_unique<InputTranslator>();

    core::EventSystem* pEventSystem = core::EventSystem::getInstance();
    pEventSystem->addListener(QUIT_GAME, this);
    pEventSystem->addListener(RESET_SCENE, this);
    pEventSystem->addListener(TOGGLE_PAUSE, this);
    for (core::EventType type : { ADD_UNIT, DELETE_UNIT_AT, DELETE_ALL_UNITS, TOGGLE_LAST_UNIT_ANIMATION, FREEZE_UNITS })
    {
        pEventSystem->addListener(type, this);
    }

    mLighting.init(2048);
    mLighting.shadowExtent = 65.0f;
    mLighting.pointLights.push_back(engine::PointLight{ {}, { 255, 150, 60, 255 }, 1.5f, 14.0f });
    mSky.init();
    mTerrain.generate(mTerrainSettings);
    mTerrain.getModel().materials[0].shader = mLighting.getShader();

    mCubeModel = LoadModelFromMesh(GenMeshCube(CUBE_HALF_SIZE * 2.0f, CUBE_HALF_SIZE * 2.0f, CUBE_HALF_SIZE * 2.0f));
    mCubeModel.materials[0].shader = mLighting.getShader();
    mBoidModel = LoadModelFromMesh(GenMeshCone(0.35f, 1.2f, 6));
    mBoidModel.materials[0].shader = mLighting.getShader();
    mUnitCube = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    mUnitCube.materials[0].shader = mLighting.getShader();
    mUnitSphere = LoadModelFromMesh(GenMeshSphere(1.0f, 12, 16));
    mUnitSphere.materials[0].shader = mLighting.getShader();

    // Convex shapes get a mesh built from their own faces, flat shaded
    mConvexShapes = {
        cyclone::ConvexCollider::wedge(cyclone::Vector3(0.6f, 0.35f, 0.4f)),
        cyclone::ConvexCollider::tetrahedron(0.9f),
        cyclone::ConvexCollider::octahedron(0.5f) };
    for (const cyclone::ConvexCollider& shape : mConvexShapes)
    {
        mConvexModels.push_back(LoadModelFromMesh(buildConvexMesh(shape)));
        mConvexModels.back().materials[0].shader = mLighting.getShader();
    }

    getCamera().position = { 0.0f, 30.0f, 55.0f };
    getCamera().target = { 0.0f, 6.0f, 0.0f };

    resetScene();
    spawnFlock();
    buildPlayground();
    createUnitSprites();
    createSounds();

#ifdef TEMPLATE_HAS_SOCKETS
    initLoopback();
#endif
#ifdef TEMPLATE_HAS_RELAY
    initRelay();
#endif
#ifdef TEMPLATE_HAS_GPU_COMPUTE
    initBalls();
#endif
}

void SandboxGame::onShutdown()
{
    // The models share the lighting shader, so detach it before UnloadModel tries to free it
    mCubeModel.materials[0].shader = Shader{};
    mBoidModel.materials[0].shader = Shader{};
    mUnitCube.materials[0].shader = Shader{};
    mUnitSphere.materials[0].shader = Shader{};
    UnloadModel(mUnitCube);
    UnloadModel(mUnitSphere);
    for (Model& model : mConvexModels)
    {
        model.materials[0].shader = Shader{};
        UnloadModel(model);
    }
    mConvexModels.clear();
    mTerrain.getModel().materials[0].shader = Shader{};
    UnloadModel(mCubeModel);
    UnloadModel(mBoidModel);
    mUnits.deleteAllUnits();
    mBuffers.clear();
    mTerrain.unload();
    mSky.unload();
    mLighting.unload();
    mpTranslator.reset();
#ifdef TEMPLATE_HAS_GPU_COMPUTE
    unloadBalls();
#endif
}

void SandboxGame::spawnFlock()
{
    mFlock = ai::Flock();
    mFlock.neighborRadius = 6.0f;
    mFlock.maxSpeed = 9.0f;
    mFlock.addRule<ai::SeparationRule>(2.0f, 2.5f);
    mFlock.addRule<ai::AlignmentRule>(1.0f);
    mFlock.addRule<ai::CohesionRule>(1.2f);
    mFlock.addRule<ai::BoundedAreaRule>(ai::Vec2{ -FLOCK_AREA, -FLOCK_AREA }, ai::Vec2{ FLOCK_AREA, FLOCK_AREA }, 12.0f, 3.0f);

    for (int i = 0; i < 40; i++)
    {
        float angle = i * 0.61f;
        float radius = 4.0f + (i % 7) * 1.5f;
        mFlock.addBoid({ std::cos(angle) * radius - 20.0f, std::sin(angle) * radius }, { std::sin(angle) * 5.0f, -std::cos(angle) * 5.0f });
    }
}

void SandboxGame::createUnitSprites()
{
    // Drawn at startup so the template ships no art: row 0 is a spinning diamond, row 1 a pulsing ring
    constexpr int FRAME = 48;
    constexpr int FRAMES = 8;
    Image sheet = GenImageColor(FRAME * FRAMES, FRAME * 2, BLANK);
    for (int i = 0; i < FRAMES; i++)
    {
        float angle = i * (PI * 0.5f / FRAMES);
        Vector2 center{ i * FRAME + FRAME * 0.5f, FRAME * 0.5f };
        Vector2 corners[4];
        for (int c = 0; c < 4; c++)
        {
            float a = angle + c * PI * 0.5f;
            corners[c] = { center.x + std::cos(a) * 18.0f, center.y + std::sin(a) * 18.0f };
        }
        ImageDrawTriangle(&sheet, corners[0], corners[2], corners[1], GOLD);
        ImageDrawTriangle(&sheet, corners[0], corners[3], corners[2], GOLD);

        int radius = 8 + (int)(10.0f * (0.5f + 0.5f * std::sin(i * 2.0f * PI / FRAMES)));
        ImageDrawCircle(&sheet, i * FRAME + FRAME / 2, FRAME + FRAME / 2, radius, SKYBLUE);
        ImageDrawCircle(&sheet, i * FRAME + FRAME / 2, FRAME + FRAME / 2, radius - 4, BLANK);
    }

    engine::GraphicsBuffer* pSheet = mBuffers.addBuffer("units", std::make_unique<engine::GraphicsBuffer>(sheet));
    UnloadImage(sheet);

    mUnitSpin = engine::Animation::fromSheet(pSheet, FRAME, FRAME, FRAMES, true, 12.0);
    mUnitPulse = engine::Animation(true, 10.0);
    for (int i = 0; i < FRAMES; i++)
    {
        mUnitPulse.addSprite(engine::Sprite(pSheet, Rectangle{ (float)(i * FRAME), (float)FRAME, (float)FRAME, (float)FRAME }));
    }
}

void SandboxGame::createSounds()
{
    // Made in code like the sprites, so the template ships no audio files
    engine::ToneSettings pop{ engine::Waveform::SINE, 520.0f, 880.0f, 0.12f, 0.5f };
    engine::ToneSettings drop{ engine::Waveform::TRIANGLE, 420.0f, 110.0f, 0.2f, 0.6f };
    engine::ToneSettings thud{ engine::Waveform::NOISE, 0.0f, 0.0f, 0.08f, 0.35f };

    for (const auto& [name, settings] : { std::pair{ "pop", pop }, std::pair{ "drop", drop }, std::pair{ "thud", thud } })
    {
        Wave wave = engine::makeTone(settings);
        getAudio().addSound(name, wave);
        UnloadWave(wave);
    }
}

void SandboxGame::handleUnitEvent(const core::Event& theEvent)
{
    switch (theEvent.getType())
    {
    case ADD_UNIT:
    {
        Vector2 position = static_cast<const UnitPositionEvent&>(theEvent).getPosition();
        engine::Unit* pUnit = mUnits.addUnit(position, mUnitSpin, mUnitPulse);
        pUnit->setActive(!mUnitsFrozen);
        getAudio().playSound("pop", 1.0f, 0.9f + 0.2f * (float)GetRandomValue(0, 100) / 100.0f);
        break;
    }
    case DELETE_UNIT_AT:
    {
        Vector2 position = static_cast<const UnitPositionEvent&>(theEvent).getPosition();
        if (engine::Unit* pUnit = mUnits.getUnitAt(position))
        {
            mUnits.deleteUnit(pUnit->getID());
            getAudio().playSound("drop");
        }
        break;
    }
    case DELETE_ALL_UNITS:
        mUnits.deleteAllUnits();
        break;
    case TOGGLE_LAST_UNIT_ANIMATION:
        if (engine::Unit* pUnit = mUnits.getLastUnit())
        {
            pUnit->toggleAnimation();
        }
        break;
    case FREEZE_UNITS:
        mUnitsFrozen = !mUnitsFrozen;
        mUnits.setAllActive(!mUnitsFrozen);
        break;
    default:
        break;
    }
}

Vector3 SandboxGame::getSunDirection() const
{
    float angle = mTimeOfDay * 2.0f * PI;
    return Vector3Normalize({ std::cos(angle), std::sin(angle), 0.35f });
}

void SandboxGame::resetScene()
{
    mBody = cyclone::RigidBody();
    mBody.setMass(1.0f);
    mBody.setPosition(cyclone::Vector3(0.0f, mTerrain.getHeight(0.0f, 0.0f) + 8.0f, 0.0f));
    mBody.setAngularVelocity(cyclone::Vector3(1.5f, 0.7f, 0.3f));

    // Solid cube inertia
    cyclone::real inertia = (cyclone::real)(1.0 / 6.0) * mBody.getMass() * (2 * CUBE_HALF_SIZE) * (2 * CUBE_HALF_SIZE);
    cyclone::Matrix3 tensor;
    tensor.setDiagonal(inertia, inertia, inertia);
    mBody.setInertiaTensor(tensor);

    mPreviousPosition = mBody.getPosition();
    mBounces = 0;
}

void SandboxGame::onFixedUpdate(double dt)
{
    if (mPaused)
    {
        return;
    }

    mPreviousPosition = mBody.getPosition();
    mBody.integrate((cyclone::real)dt);
    if (mShowFlock)
    {
        mFlock.update((float)dt);
    }
    stepPlayground(dt);
#ifdef TEMPLATE_HAS_GPU_COMPUTE
    stepBalls(dt);
#endif

    // Placeholder contact with the terrain under the cube, until real collision handling goes in
    cyclone::Vector3 position = mBody.getPosition();
    cyclone::Vector3 velocity = mBody.getLinearVelocity();
    float groundY = mTerrain.getHeight((float)position.x, (float)position.z) + CUBE_HALF_SIZE;
    if (position.y < groundY && velocity.y < 0)
    {
        // Slow impacts are resting contact, not bounces
        if (velocity.y < -0.5f)
        {
            mBounces++;
            getAudio().playSound("thud", std::min(1.0f, (float)-velocity.y / 15.0f));
            velocity.y = -velocity.y * mRestitution;
        }
        else
        {
            velocity.y = 0;
        }
        position.y = groundY;
        mBody.setPosition(position);
        mBody.setLinearVelocity(velocity);
    }
}

void SandboxGame::onUpdate(double frameDt, double alpha)
{
    mFlyCamera.update(getCamera(), (float)frameDt);
    mUnits.update(frameDt);
    mRenderAlpha = (float)alpha;
#ifdef TEMPLATE_HAS_GPU_COMPUTE
    updateBalls();
#endif

    // Draw between the last two physics states so motion stays smooth at any frame rate
    mRenderPosition = Vector3Lerp(engine::toRaylib(mPreviousPosition), engine::toRaylib(mBody.getPosition()), (float)alpha);

#ifdef TEMPLATE_HAS_SOCKETS
    pollLoopback();
#endif
#ifdef TEMPLATE_HAS_RELAY
    mRelay.poll();
#endif
}

void SandboxGame::onRenderOffscreen()
{
    Vector3 sun = getSunDirection();
    float daylight = Clamp(sun.y * 4.0f, 0.0f, 1.0f);
    mLighting.sun.direction = sun;
    mLighting.sun.intensity = daylight;
    mLighting.ambient = 0.06f + 0.22f * daylight;
    mLighting.pointLights[0].position = Vector3Add(mRenderPosition, { 0.0f, 1.5f, 0.0f });

    // No sun shadows at night; the cube's point light still lights the scene
    mLighting.shadowsEnabled = mShadows && daylight > 0.0f;
    mLighting.renderShadows({ 0.0f, 0.0f, 0.0f }, [this]()
    {
        drawScene();
    });
    mLighting.apply(getCamera());
}

void SandboxGame::drawScene()
{
    Matrix rotation = QuaternionToMatrix(engine::toRaylib(mBody.getOrientation()));
    mCubeModel.transform = MatrixMultiply(rotation, MatrixTranslate(mRenderPosition.x, mRenderPosition.y, mRenderPosition.z));
    DrawModel(mTerrain.getModel(), Vector3Zero(), 1.0f, WHITE);
    DrawModel(mCubeModel, Vector3Zero(), 1.0f, ORANGE);
    drawPlayground();

    if (!mShowFlock)
    {
        return;
    }
    for (const ai::Boid& boid : mFlock.getBoids())
    {
        // Cones point up +Y, so tip them onto +Z, then turn to face the direction of travel
        float yaw = std::atan2(boid.velocity.x, boid.velocity.y);
        float height = mTerrain.getHeight(boid.position.x, boid.position.y) + FLOCK_ALTITUDE;
        Matrix orient = MatrixMultiply(MatrixRotateX(PI * 0.5f), MatrixRotateY(yaw));
        mBoidModel.transform = MatrixMultiply(orient, MatrixTranslate(boid.position.x, height, boid.position.y));
        DrawModel(mBoidModel, Vector3Zero(), 1.0f, SKYBLUE);
    }
}

PlaygroundBody* SandboxGame::addPlaygroundBody(std::unique_ptr<PlaygroundBody> pBody)
{
    pBody->pCollider->pBody = &pBody->body;
    pBody->previousPosition = pBody->body.getPosition();
    pBody->previousOrientation = pBody->body.getOrientation();
    mPhysics.bodies.push_back(&pBody->body);
    mPhysics.colliders.push_back(pBody->pCollider.get());
    mPhysics.forces.add(&pBody->body, &mExplosion);
    mPlayBodies.push_back(std::move(pBody));
    return mPlayBodies.back().get();
}

void SandboxGame::removePlaygroundBody(PlaygroundBody* pBody)
{
    auto& bodies = mPhysics.bodies;
    auto& colliders = mPhysics.colliders;
    bodies.erase(std::remove(bodies.begin(), bodies.end(), &pBody->body), bodies.end());
    colliders.erase(std::remove(colliders.begin(), colliders.end(), pBody->pCollider.get()), colliders.end());
    mPhysics.forces.remove(&pBody->body, &mExplosion);
    mPlayBodies.erase(std::remove_if(mPlayBodies.begin(), mPlayBodies.end(), [&](const std::unique_ptr<PlaygroundBody>& p)
    {
        return p.get() == pBody;
    }), mPlayBodies.end());
}

PlaygroundBody* SandboxGame::addPlaygroundBox(Vector3 position, Vector3 halfSize, float mass)
{
    auto pBody = std::make_unique<PlaygroundBody>();
    pBody->shape = PlaygroundShape::BOX;
    pBody->body.setPosition(engine::toCyclone(position));
    pBody->body.setMass(mass);
    cyclone::Matrix3 tensor;
    tensor.setBlockInertiaTensor(engine::toCyclone(halfSize), mass);
    pBody->body.setInertiaTensor(tensor);
    pBody->pCollider = std::make_unique<cyclone::BoxCollider>(engine::toCyclone(halfSize));
    pBody->color = mPlayBodies.size() % 2 == 0 ? Color{ 214, 140, 70, 255 } : Color{ 190, 110, 60, 255 };
    return addPlaygroundBody(std::move(pBody));
}

PlaygroundBody* SandboxGame::addPlaygroundBall(Vector3 position, float radius, float mass)
{
    auto pBody = std::make_unique<PlaygroundBody>();
    pBody->shape = PlaygroundShape::BALL;
    pBody->body.setPosition(engine::toCyclone(position));
    pBody->body.setMass(mass);
    const cyclone::real inertia = (cyclone::real)0.4 * mass * radius * radius;
    pBody->body.setInertiaTensor(cyclone::Matrix3(inertia, 0, 0, 0, inertia, 0, 0, 0, inertia));
    pBody->pCollider = std::make_unique<cyclone::SphereCollider>(radius);
    pBody->color = GOLD;
    return addPlaygroundBody(std::move(pBody));
}

PlaygroundBody* SandboxGame::addPlaygroundConvex(int shapeIndex, Vector3 position, float mass)
{
    const cyclone::ConvexCollider& shape = mConvexShapes[shapeIndex];
    auto pBody = std::make_unique<PlaygroundBody>();
    pBody->shape = PlaygroundShape::CONVEX;
    pBody->convexModel = shapeIndex;
    pBody->body.setPosition(engine::toCyclone(position));
    pBody->body.setMass(mass);
    pBody->body.setInertiaTensor(shape.computeInertiaTensor(mass));
    pBody->pCollider = std::make_unique<cyclone::ConvexCollider>(shape);
    const Color colors[3] = { Color{ 120, 190, 120, 255 }, Color{ 200, 90, 160, 255 }, Color{ 110, 150, 230, 255 } };
    pBody->color = colors[shapeIndex % 3];
    return addPlaygroundBody(std::move(pBody));
}

void SandboxGame::buildPlayground()
{
    mPhysics.bodies.clear();
    mPhysics.colliders.clear();
    mPhysics.contactGenerators.clear();
    mPhysics.forces.clear();
    mPhysics.clearCoherence();
    mPlayBodies.clear();
    mJoints.clear();
    mThrownBalls = 0;
    mExplosion = cyclone::Explosion();
    mExplosion.shockwaveSpeed = 15.0f;
    mExplosion.shockwaveThickness = 3.0f;
    mExplosion.peakConcussionForce = 45.0f;
    mExplosion.concussionDuration = 0.8f;
    mExplosion.peakConvectionForce = 15.0f;
    mExplosion.chimneyRadius = 3.0f;

    // A floating platform: a static box with no body, top face at the centre's height
    const float x = 26.0f;
    const float z = -6.0f;
    mPlaygroundCenter = { x, mTerrain.getHeight(x, z) + 5.0f, z };
    mPlatform.halfSize = cyclone::Vector3(PLATFORM_HALF_SIZE, 0.5f, PLATFORM_HALF_SIZE);
    mPlatform.pBody = nullptr;
    mPlatform.offset = cyclone::Matrix4();
    mPlatform.offset.data[3] = mPlaygroundCenter.x;
    mPlatform.offset.data[7] = mPlaygroundCenter.y - 0.5f;
    mPlatform.offset.data[11] = mPlaygroundCenter.z;
    mPhysics.colliders.push_back(&mPlatform);

    // A hidden floor under the platform catches anything knocked off, so it can come to rest and sleep
    mCatchFloor = cyclone::PlaneCollider(cyclone::Vector3(0, 1, 0), mPlaygroundCenter.y - 12.0f);
    mPhysics.colliders.push_back(&mCatchFloor);

    const Vector3 c = mPlaygroundCenter;
    const Vector3 cubeHalf = { 0.5f, 0.5f, 0.5f };

    // A pyramid of crates
    for (int row = 0; row < 4; row++)
    {
        for (int i = 0; i < 4 - row; i++)
        {
            addPlaygroundBox({ c.x - 3.0f + i * 1.02f + row * 0.51f, c.y + 0.5f + row * 1.0f, c.z - 2.5f }, cubeHalf, 1.0f);
        }
    }

    // A tall tower
    for (int i = 0; i < 6; i++)
    {
        addPlaygroundBox({ c.x + 3.5f, c.y + 0.4f + i * 0.8f, c.z + 2.5f }, { 0.4f, 0.4f, 0.4f }, 0.6f);
    }

    // A row of balls
    for (int i = 0; i < 5; i++)
    {
        addPlaygroundBall({ c.x - 4.0f + i * 1.0f, c.y + 0.4f, c.z + 4.0f }, 0.4f, 1.0f);
    }

    // Convex shapes dropped in a loose heap
    for (int i = 0; i < 6; i++)
    {
        PlaygroundBody* pShape = addPlaygroundConvex(i % 3, { c.x + 1.0f + (i % 3) * 1.3f, c.y + 1.0f + (i / 3) * 1.4f, c.z - 4.5f }, 0.8f);
        pShape->body.setOrientation(cyclone::Quaternion(0.9f, 0.3f * (i % 2), 0.2f, 0.1f * i));
    }

    // A glass block that shatters when hit hard, and its pieces can break once more
    PlaygroundBody* pGlass = addPlaygroundBox({ c.x - 0.5f, c.y + 0.75f, c.z + 1.5f }, { 0.75f, 0.75f, 0.75f }, 2.0f);
    pGlass->color = Color{ 170, 220, 240, 255 };
    pGlass->fractureLevelsLeft = 2;

    // A chain of links hanging from a point above the platform's corner
    const Vector3 linkHalf = { 0.15f, 0.35f, 0.15f };
    const Vector3 anchor = { c.x + 5.0f, c.y + 7.5f, c.z - 5.0f };
    PlaygroundBody* pPrevious = nullptr;
    for (int i = 0; i < 6; i++)
    {
        // Each link starts off to the side, so the chain swings when it starts
        Vector3 position = { anchor.x + 0.75f * (i + 0.5f), anchor.y - 0.1f * i, anchor.z };
        PlaygroundBody* pLink = addPlaygroundBox(position, linkHalf, 0.4f);
        pLink->color = SKYBLUE;
        pLink->body.setOrientation(cyclone::Quaternion(0.7071f, 0.0f, 0.0f, 0.7071f));
        pLink->previousOrientation = pLink->body.getOrientation();
        auto pJoint = std::make_unique<cyclone::Joint>();
        if (pPrevious == nullptr)
        {
            pJoint->set(&pLink->body, cyclone::Vector3(0, linkHalf.y, 0), nullptr, engine::toCyclone(anchor), 0.01f);
        }
        else
        {
            pJoint->set(&pLink->body, cyclone::Vector3(0, linkHalf.y, 0), &pPrevious->body, cyclone::Vector3(0, -linkHalf.y, 0), 0.01f);
        }
        mPhysics.contactGenerators.push_back(pJoint.get());
        mJoints.push_back(std::move(pJoint));
        pPrevious = pLink;
    }
}

void SandboxGame::stepPlayground(double dt)
{
    // Remember where everything was, so drawing can blend toward where it ends up
    for (auto& pBody : mPlayBodies)
    {
        pBody->previousPosition = pBody->body.getPosition();
        pBody->previousOrientation = pBody->body.getOrientation();
        pBody->velocityBeforeStep = pBody->body.getLinearVelocity();
    }
    mPhysics.broadPhase = (cyclone::World::BroadPhase)mBroadPhase;
    mPhysics.resolverType = (cyclone::World::Resolver)mResolver;
    mExplosion.advance((cyclone::real)dt);
    mPhysics.runPhysics((cyclone::real)dt);
    fractureHitBodies();
}

void SandboxGame::fractureHitBodies()
{
    std::vector<PlaygroundBody*> broken;
    for (auto& pBody : mPlayBodies)
    {
        if (pBody->fractureLevelsLeft > 0 &&
            (pBody->body.getLinearVelocity() - pBody->velocityBeforeStep).magnitude() > FRACTURE_SPEED)
        {
            broken.push_back(pBody.get());
        }
    }

    for (PlaygroundBody* pBody : broken)
    {
        if (mPlayBodies.size() + 8 > (std::size_t)MAX_PLAYGROUND_BODIES)
        {
            break;
        }
        // Break at the deepest contact touching it
        cyclone::Vector3 impact = pBody->body.getPosition();
        cyclone::real deepest = -1;
        for (unsigned i = 0; i < mPhysics.getContactCount(); i++)
        {
            const cyclone::Contact& contact = mPhysics.getContacts()[i];
            if ((contact.body[0] == &pBody->body || contact.body[1] == &pBody->body) && contact.penetration > deepest)
            {
                deepest = contact.penetration;
                impact = contact.point;
            }
        }
        const cyclone::Vector3 halfSize = static_cast<const cyclone::BoxCollider&>(*pBody->pCollider).halfSize;
        const auto pieces = cyclone::fractureBox(pBody->body, halfSize, pBody->body.getPointInLocalSpace(impact), 0.08f, 1.5f);
        const int levelsLeft = pBody->fractureLevelsLeft - 1;
        const Color color = pBody->color;
        const cyclone::Vector3 previousPosition = pBody->previousPosition;
        const cyclone::Quaternion previousOrientation = pBody->previousOrientation;
        const cyclone::Vector3 bodyPosition = pBody->body.getPosition();
        removePlaygroundBody(pBody);
        for (const cyclone::BoxFragment& piece : pieces)
        {
            auto pPiece = std::make_unique<PlaygroundBody>();
            pPiece->body.setPosition(piece.position);
            pPiece->body.setOrientation(piece.orientation);
            pPiece->body.setMass(piece.mass);
            pPiece->body.setInertiaTensor(piece.inertiaTensor);
            pPiece->body.setLinearVelocity(piece.linearVelocity);
            pPiece->body.setAngularVelocity(piece.angularVelocity);
            pPiece->pCollider = std::make_unique<cyclone::BoxCollider>(piece.halfSize);
            pPiece->color = color;
            pPiece->fractureLevelsLeft = levelsLeft;
            PlaygroundBody* pAdded = addPlaygroundBody(std::move(pPiece));
            // Drawn from where the whole block was, so the break doesn't jump
            pAdded->previousPosition = previousPosition + (piece.position - bodyPosition);
            pAdded->previousOrientation = previousOrientation;
        }
    }
}

void SandboxGame::throwBall()
{
    if (mThrownBalls >= MAX_THROWN_BALLS || mPlayBodies.size() >= (std::size_t)MAX_PLAYGROUND_BODIES)
    {
        return;
    }
    const Camera3D& camera = getCamera();
    const Vector3 forward = Vector3Normalize(Vector3Subtract(camera.target, camera.position));
    PlaygroundBody* pBall = addPlaygroundBall(Vector3Add(camera.position, Vector3Scale(forward, 2.0f)), 0.35f, 3.0f);
    pBall->body.setLinearVelocity(engine::toCyclone(Vector3Scale(forward, 35.0f)));
    mThrownBalls++;
}

void SandboxGame::dropBallOnGlass()
{
    // The biggest piece of glass left, so the first drop shatters the block and later ones break the pieces
    PlaygroundBody* pTarget = nullptr;
    for (auto& pBody : mPlayBodies)
    {
        if (pBody->fractureLevelsLeft > 0 && (pTarget == nullptr || pBody->fractureLevelsLeft > pTarget->fractureLevelsLeft))
        {
            pTarget = pBody.get();
        }
    }
    if (pTarget == nullptr || mPlayBodies.size() >= (std::size_t)MAX_PLAYGROUND_BODIES)
    {
        return;
    }
    const Vector3 above = Vector3Add(engine::toRaylib(pTarget->body.getPosition()), { 0.1f, 4.0f, 0.05f });
    PlaygroundBody* pBall = addPlaygroundBall(above, 0.35f, 4.0f);
    pBall->body.setLinearVelocity(cyclone::Vector3(0, -12, 0));
}

Matrix SandboxGame::interpolatedTransform(const PlaygroundBody& body) const
{
    // Between the pose before the last step and the pose after it
    const Vector3 position = Vector3Lerp(engine::toRaylib(body.previousPosition), engine::toRaylib(body.body.getPosition()), mRenderAlpha);
    const Quaternion orientation = QuaternionSlerp(engine::toRaylib(body.previousOrientation), engine::toRaylib(body.body.getOrientation()), mRenderAlpha);
    return MatrixMultiply(QuaternionToMatrix(orientation), MatrixTranslate(position.x, position.y, position.z));
}

void SandboxGame::drawPlayground()
{
    const Vector3 platformSize = { PLATFORM_HALF_SIZE * 2.0f, 1.0f, PLATFORM_HALF_SIZE * 2.0f };
    mUnitCube.transform = MatrixMultiply(MatrixScale(platformSize.x, platformSize.y, platformSize.z),
        MatrixTranslate(mPlaygroundCenter.x, mPlaygroundCenter.y - 0.5f, mPlaygroundCenter.z));
    DrawModel(mUnitCube, Vector3Zero(), 1.0f, Color{ 150, 150, 160, 255 });

    // Sleeping bodies are drawn grey, so you can watch them settle
    for (const auto& pBody : mPlayBodies)
    {
        const Matrix pose = interpolatedTransform(*pBody);
        const Color asleep = { (unsigned char)(pBody->color.r * 0.3f + 84), (unsigned char)(pBody->color.g * 0.3f + 88),
            (unsigned char)(pBody->color.b * 0.3f + 98), 255 };
        const Color color = pBody->body.isAwake() ? pBody->color : asleep;
        switch (pBody->shape)
        {
        case PlaygroundShape::BOX:
        {
            const cyclone::Vector3 h = static_cast<const cyclone::BoxCollider&>(*pBody->pCollider).halfSize;
            mUnitCube.transform = MatrixMultiply(MatrixScale((float)h.x * 2.0f, (float)h.y * 2.0f, (float)h.z * 2.0f), pose);
            DrawModel(mUnitCube, Vector3Zero(), 1.0f, color);
            break;
        }
        case PlaygroundShape::BALL:
        {
            const float r = (float)static_cast<const cyclone::SphereCollider&>(*pBody->pCollider).radius;
            mUnitSphere.transform = MatrixMultiply(MatrixScale(r, r, r), pose);
            DrawModel(mUnitSphere, Vector3Zero(), 1.0f, color);
            break;
        }
        case PlaygroundShape::CONVEX:
        {
            Model& model = mConvexModels[pBody->convexModel];
            model.transform = pose;
            DrawModel(model, Vector3Zero(), 1.0f, color);
            break;
        }
        }
    }
}

void SandboxGame::drawPlaygroundContacts()
{
    for (unsigned i = 0; i < mPhysics.getContactCount(); i++)
    {
        const cyclone::Contact& contact = mPhysics.getContacts()[i];
        const Vector3 point = engine::toRaylib(contact.point);
        DrawSphere(point, 0.06f, RED);
        DrawLine3D(point, Vector3Add(point, Vector3Scale(engine::toRaylib(contact.normal), 0.5f)), RED);
    }
    for (const auto& pJoint : mJoints)
    {
        DrawLine3D(engine::toRaylib(pJoint->getWorldPosition(0)), engine::toRaylib(pJoint->getWorldPosition(1)), YELLOW);
    }
}

void SandboxGame::drawPlaygroundGui()
{
    ImGui::Separator();
    ImGui::Text("Physics: %zu bodies, %u awake, %u contacts", mPhysics.bodies.size(), mPhysics.getAwakeCount(), mPhysics.getContactCount());
    ImGui::Text("Pairs tested %u, settled by cached axis %u", mPhysics.getPairsTested(), mPhysics.getCoherenceHits());
    if (ImGui::Button("Reset playground"))
    {
        buildPlayground();
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(mThrownBalls >= MAX_THROWN_BALLS);
    if (ImGui::Button("Throw ball"))
    {
        throwBall();
    }
    ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Explode"))
    {
        mExplosion.detonate(cyclone::Vector3(mPlaygroundCenter.x - 1.5f, mPlaygroundCenter.y - 0.3f, mPlaygroundCenter.z - 2.5f));
    }
    if (ImGui::Button("Drop ball on glass"))
    {
        dropBallOnGlass();
    }
    const char* broadPhases[] = { "All pairs", "Bounding volume hierarchy", "Multiresolution map", "BSP tree" };
    ImGui::Combo("Broad phase", &mBroadPhase, broadPhases, 4);
    ImGui::RadioButton("Sequential", &mResolver, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Simultaneous (LCP)", &mResolver, 1);
    float friction = (float)mPhysics.friction;
    if (ImGui::SliderFloat("Friction", &friction, 0.0f, 1.0f))
    {
        mPhysics.friction = friction;
    }
    float bounce = (float)mPhysics.restitution;
    if (ImGui::SliderFloat("Bounciness", &bounce, 0.0f, 1.0f))
    {
        mPhysics.restitution = bounce;
    }
    ImGui::Checkbox("Coherence", &mPhysics.useCoherence);
    ImGui::SameLine();
    ImGui::Checkbox("Show contacts", &mShowContacts);
}

void SandboxGame::onDraw3D()
{
    mSky.draw(getSunDirection());
    drawScene();

    // Drawn as lines rather than DrawModelWires, because WebGL has no wireframe polygon mode
    rlPushMatrix();
    rlMultMatrixf(MatrixToFloatV(mCubeModel.transform).v);
    DrawCubeWires(Vector3Zero(), CUBE_HALF_SIZE * 2.0f, CUBE_HALF_SIZE * 2.0f, CUBE_HALF_SIZE * 2.0f, BLACK);
    rlPopMatrix();
    if (mShowContacts)
    {
        drawPlaygroundContacts();
    }

#ifdef TEMPLATE_HAS_GPU_COMPUTE
    drawBalls();
#endif
}

void SandboxGame::onDraw2D()
{
    mUnits.draw();
    DrawText("Right mouse + WASD fly | Left click add unit, middle delete, Enter toggle, F freeze, Backspace clear | Space pause | R reset | Esc quit",
        12, GetScreenHeight() - 28, 18, RAYWHITE);
}

void SandboxGame::onGui()
{
    ImGui::Begin("Sandbox");
    ImGui::Text("FPS: %d", GetFPS());
    ImGui::Text("Physics: %.0f Hz, sim time %.2f s", getConfig().fixedHz, getTime());
    ImGui::Text("Bounces: %d", mBounces);
    ImGui::SliderFloat("Restitution", &mRestitution, 0.0f, 1.0f);
    ImGui::Checkbox("Paused", &mPaused);
    if (ImGui::Button("Reset"))
    {
        resetScene();
    }
    ImGui::Separator();
    ImGui::SliderFloat("Time of day", &mTimeOfDay, 0.0f, 1.0f);
    ImGui::Checkbox("Shadows", &mShadows);
    ImGui::SameLine();
    ImGui::Checkbox("Flock", &mShowFlock);
    ImGui::SliderInt("Terrain seed", &mTerrainSettings.seed, 0, 9999);
    ImGui::SliderFloat("Terrain height", &mTerrainSettings.height, 0.0f, 40.0f);
    if (ImGui::Button("Regenerate terrain"))
    {
        mTerrain.generate(mTerrainSettings);
        mTerrain.getModel().materials[0].shader = mLighting.getShader();
        resetScene();
    }
    ImGui::Separator();
    drawPlaygroundGui();
    ImGui::Separator();
    ImGui::Text("Units: %zu%s", mUnits.getUnitCount(), mUnitsFrozen ? " (frozen)" : "");
    ImGui::Text("Tracked allocations: %zu", core::MemoryTracker::getInstance().getNumAllocations());

    ImGui::Separator();
    if (getAudio().isReady())
    {
        float volume = getAudio().getMasterVolume();
        if (ImGui::SliderFloat("Volume", &volume, 0.0f, 1.0f))
        {
            getAudio().setMasterVolume(volume);
        }
    }
    else
    {
        ImGui::TextDisabled("No audio device");
    }

#ifdef TEMPLATE_HAS_SOCKETS
    drawSocketGui();
#endif
#ifdef TEMPLATE_HAS_RELAY
    drawRelayGui();
#endif
#ifdef TEMPLATE_HAS_GPU_COMPUTE
    drawBallGui();
#endif
    ImGui::End();
}

void SandboxGame::handleEvent(const core::Event& theEvent)
{
    if (theEvent.getType() >= ADD_UNIT && theEvent.getType() <= FREEZE_UNITS)
    {
        handleUnitEvent(theEvent);
        return;
    }

    switch (theEvent.getType())
    {
    case QUIT_GAME:
        requestQuit();
        break;
    case RESET_SCENE:
        resetScene();
#ifdef TEMPLATE_HAS_RELAY
        {
            // Tell the peer too, so both scenes restart together
            const std::uint8_t resetMessage = 1;
            mRelay.send(&resetMessage, 1);
        }
#endif
        break;
    case TOGGLE_PAUSE:
        mPaused = !mPaused;
        break;
    default:
        break;
    }
}

#ifdef TEMPLATE_HAS_SOCKETS
void SandboxGame::initLoopback()
{
    if (!mServer.open() || !mClient.open())
    {
        TraceLog(LOG_WARNING, "UDP loopback: could not open sockets");
        return;
    }

    // The server echoes every ping back as a pong
    mServerDispatcher.setHandler(PING, [this](net::BitReader& reader)
    {
        PingMessage ping;
        if (serializePing(reader, ping))
        {
            auto pong = net::makePacket(PONG, [&](net::BitWriter& writer)
            {
                serializePing(writer, ping);
            });
            mServer.sendTo(net::Address{ "127.0.0.1", mClient.getLocalPort() }, pong.data(), pong.size());
        }
    });

    mClientDispatcher.setHandler(PONG, [this](net::BitReader& reader)
    {
        PingMessage pong;
        if (serializePing(reader, pong))
        {
            mPongsReceived++;
            mLastRoundTripMS = (GetTime() - pong.sentAt) * 1000.0;
        }
    });

    getCallbacks().addTimed(0.5, [this]()
    {
        PingMessage ping{ mPingsSent++, (float)GetTime() };
        auto packet = net::makePacket(PING, [&](net::BitWriter& writer)
        {
            serializePing(writer, ping);
        });
        mClient.sendTo(net::Address{ "127.0.0.1", mServer.getLocalPort() }, packet.data(), packet.size());
    }, true);

    if (mTcpListener.open())
    {
        mTcpClient.connect(net::Address{ "127.0.0.1", mTcpListener.getLocalPort() });
    }
}

void SandboxGame::drawSocketGui()
{
    ImGui::Separator();
    ImGui::Text("UDP loopback on port %u", (unsigned int)mServer.getLocalPort());
    ImGui::Text("Pings %u, pongs %u, last RTT %.2f ms", mPingsSent, mPongsReceived, mLastRoundTripMS);

    ImGui::Separator();
    ImGui::Text("TCP loopback chat: %s", mTcpClient.isConnected() ? "connected" : "disconnected");
    bool submitted = ImGui::InputText("##chat", mChatInput, sizeof(mChatInput), ImGuiInputTextFlags_EnterReturnsTrue);
    ImGui::SameLine();
    if ((ImGui::Button("Send") || submitted) && mChatInput[0] != '\0')
    {
        mTcpClient.send((const std::uint8_t*)mChatInput, std::strlen(mChatInput));
        mChatInput[0] = '\0';
    }
    for (const std::string& line : mChatLog)
    {
        ImGui::TextUnformatted(line.c_str());
    }

    ImGui::Separator();
    if (ImGui::Checkbox("Host on LAN", &mHostOnLan))
    {
        if (mHostOnLan)
        {
            std::string info = getConfig().title + " tcp=" + std::to_string(mTcpListener.getLocalPort());
            mHostOnLan = mDiscovery.host(info);
        }
        else
        {
            mDiscovery.stopHosting();
        }
    }
    ImGui::SameLine();
    if (ImGui::Button("Search LAN"))
    {
        mDiscovery.clearServers();
        mDiscovery.search();
    }
    for (const net::LanServer& server : mDiscovery.getServers())
    {
        ImGui::Text("%s:%u  %s", server.address.host.c_str(), (unsigned int)server.address.port, server.info.c_str());
    }
}

void SandboxGame::pollLoopback()
{
    if (mpTcpServerSide == nullptr)
    {
        mpTcpServerSide = mTcpListener.accept();
    }
    if (mpTcpServerSide != nullptr)
    {
        // The server side answers every chat line, standing in for a real chat server
        while (auto message = mpTcpServerSide->receive())
        {
            std::string reply = "[Server]: got \"" + std::string(message->begin(), message->end()) + "\"";
            mpTcpServerSide->send((const std::uint8_t*)reply.data(), reply.size());
        }
    }
    while (auto message = mTcpClient.receive())
    {
        mChatLog.emplace_back(message->begin(), message->end());
        if (mChatLog.size() > 6)
        {
            mChatLog.erase(mChatLog.begin());
        }
    }

    mDiscovery.poll();

    std::uint8_t buffer[1500];
    net::Address from;
    while (auto size = mServer.receive(from, buffer, sizeof(buffer)))
    {
        mServerDispatcher.dispatch(buffer, *size);
    }
    while (auto size = mClient.receive(from, buffer, sizeof(buffer)))
    {
        mClientDispatcher.dispatch(buffer, *size);
    }
}
#endif

#ifdef TEMPLATE_HAS_GPU_COMPUTE
void SandboxGame::initBalls()
{
    Shader shader = engine::loadPortableShader(BALL_VERTEX_SHADER, BALL_FRAGMENT_SHADER);
    // raylib 5.5 reads the per-instance matrix attribute from the model matrix slot
    shader.locs[SHADER_LOC_MATRIX_MODEL] = GetShaderLocationAttrib(shader, "instanceTransform");
    mBallSunLocation = GetShaderLocation(shader, "sunDirection");
    mBallRangeLocation = GetShaderLocation(shader, "heightRange");
    mBallMaterial = LoadMaterialDefault();
    mBallMaterial.shader = shader;
    mBallMesh = GenMeshSphere(1.0f, 8, 12);

    // Runs on the CPU until WebGPU has started, then moves to the GPU
    spawnBalls();
    mGpu.start();
}

void SandboxGame::unloadBalls()
{
    UnloadMesh(mBallMesh);
    UnloadMaterial(mBallMaterial); // Also frees its shader
}

bool SandboxGame::ballsOnGpu() const
{
    return mWantGpu && mGpu.isReady() && mGpuBalls.isReady();
}

const std::vector<gpu::Float4>& SandboxGame::getBallSpheres() const
{
    return ballsOnGpu() ? mGpuBalls.getSpheres() : mCpuBalls.getSpheres();
}

void SandboxGame::spawnBalls()
{
    // A lattice through the top of the box, spaced so every ball fits
    const float half = BALL_BOX_HALF_SIZE;
    const float spacing = std::cbrt((2.0f * half) * (2.0f * half) * (1.4f * half) / (float)mBallCount);
    const float maxRadius = std::min(0.25f, spacing * 0.45f);
    const int perRow = std::max(1, (int)((2.0f * half) / spacing));

    std::mt19937 random(1234);
    std::uniform_real_distribution<float> unit(0.0f, 1.0f);
    std::vector<gpu::Float4> spheres;
    std::vector<gpu::Float4> velocities;
    spheres.reserve(mBallCount);
    velocities.reserve(mBallCount);
    for (int i = 0; i < mBallCount; i++)
    {
        const int x = i % perRow;
        const int z = (i / perRow) % perRow;
        const int y = i / (perRow * perRow);
        const float radius = maxRadius * (0.6f + 0.4f * unit(random));
        spheres.push_back({ -half + spacing * (x + 0.5f), half - spacing * (y + 0.5f), -half + spacing * (z + 0.5f), radius });
        const float inverseMass = std::pow(maxRadius / radius, 3.0f);
        velocities.push_back({ (unit(random) - 0.5f) * 4.0f, 0.0f, (unit(random) - 0.5f) * 4.0f, inverseMass });
    }

    mCpuBalls.reset(spheres, velocities, half);
    if (mGpuBalls.isReady())
    {
        mGpuBalls.reset(spheres, velocities, half);
    }
}

void SandboxGame::setBallsOnGpu(bool useGpu)
{
    const bool wasOnGpu = ballsOnGpu();
    mWantGpu = useGpu;
    if (wasOnGpu == ballsOnGpu())
    {
        return;
    }
    // Hand the current state across, so switching doesn't restart the simulation
    if (ballsOnGpu())
    {
        mGpuBalls.reset(mCpuBalls.getSpheres(), mCpuBalls.getVelocities(), BALL_BOX_HALF_SIZE);
    }
    else
    {
        mCpuBalls.reset(mGpuBalls.getSpheres(), mGpuBalls.getVelocities(), BALL_BOX_HALF_SIZE);
    }
}

void SandboxGame::stepBalls(double dt)
{
    if (ballsOnGpu())
    {
        mGpuBalls.step((float)dt, mBallSettings);
        return;
    }
    auto start = std::chrono::steady_clock::now();
    mCpuBalls.step((float)dt, mBallSettings);
    double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    mCpuStepMS = mCpuStepMS * 0.9 + ms * 0.1;
}

void SandboxGame::updateBalls()
{
    mGpu.poll();
    if (!mGpuInitDone && mGpu.getState() == gpu::ContextState::READY)
    {
        mGpuInitDone = true;
        if (!mGpuBalls.init(mGpu))
        {
            TraceLog(LOG_WARNING, "GPU: could not build the ball simulation, staying on the CPU");
        }
        else if (mWantGpu)
        {
            TraceLog(LOG_INFO, "GPU: balls now run on %s", mGpu.getStatus().c_str());
            mGpuBalls.reset(mCpuBalls.getSpheres(), mCpuBalls.getVelocities(), BALL_BOX_HALF_SIZE);
        }
    }
    else if (!mGpuInitDone && mGpu.getState() == gpu::ContextState::UNAVAILABLE)
    {
        mGpuInitDone = true;
        TraceLog(LOG_WARNING, "GPU: %s, balls stay on the CPU", mGpu.getStatus().c_str());
    }
    if (ballsOnGpu())
    {
        mGpuBalls.poll();
        mGpuBalls.requestReadback();
    }
}

void SandboxGame::drawBalls()
{
    const std::vector<gpu::Float4>& spheres = getBallSpheres();
    mBallTransforms.resize(spheres.size());
    for (std::size_t i = 0; i < spheres.size(); i++)
    {
        const gpu::Float4& s = spheres[i];
        Matrix& m = mBallTransforms[i];
        m = Matrix{};
        m.m0 = s.w;
        m.m5 = s.w;
        m.m10 = s.w;
        m.m12 = BALL_BOX_CENTER.x + s.x;
        m.m13 = BALL_BOX_CENTER.y + s.y;
        m.m14 = BALL_BOX_CENTER.z + s.z;
        m.m15 = 1.0f;
    }

    Vector3 sun = getSunDirection();
    float range[2] = { BALL_BOX_CENTER.y - BALL_BOX_HALF_SIZE, BALL_BOX_CENTER.y + BALL_BOX_HALF_SIZE };
    SetShaderValue(mBallMaterial.shader, mBallSunLocation, &sun, SHADER_UNIFORM_VEC3);
    SetShaderValue(mBallMaterial.shader, mBallRangeLocation, range, SHADER_UNIFORM_VEC2);
    if (!mBallTransforms.empty())
    {
        DrawMeshInstanced(mBallMesh, mBallMaterial, mBallTransforms.data(), (int)mBallTransforms.size());
    }
    const float size = BALL_BOX_HALF_SIZE * 2.0f;
    DrawCubeWires(BALL_BOX_CENTER, size, size, size, Color{ 220, 220, 230, 255 });
}

void SandboxGame::drawBallGui()
{
    ImGui::Separator();
    ImGui::Text("Balls: %zu", getBallSpheres().size());
    ImGui::TextWrapped("WebGPU: %s", mGpu.getStatus().c_str());

    const bool onGpu = ballsOnGpu();
    if (ImGui::RadioButton("CPU", !onGpu))
    {
        setBallsOnGpu(false);
    }
    ImGui::SameLine();
    ImGui::BeginDisabled(!mGpu.isReady() || !mGpuBalls.isReady());
    if (ImGui::RadioButton("GPU", onGpu))
    {
        setBallsOnGpu(true);
    }
    ImGui::EndDisabled();

    if (ballsOnGpu())
    {
        unsigned long long behind = mGpuBalls.getStepsBehind();
        ImGui::Text("GPU, drawing %llu %s behind", behind, behind == 1 ? "step" : "steps");
    }
    else
    {
        ImGui::Text("CPU step: %.2f ms", mCpuStepMS);
    }
    const gpu::BallStats& stats = ballsOnGpu() ? mGpuBalls.getStats() : mCpuBalls.getStats();
    ImGui::Text("Contacts: %u balls, %u walls", stats.ballContacts, stats.wallContacts);

    ImGui::SliderInt("Ball count", &mBallCount, 500, 200000, "%d", ImGuiSliderFlags_Logarithmic);
    ImGui::SliderInt("Iterations", &mBallSettings.iterations, 1, gpu::kMaxIterations);
    ImGui::SliderFloat("Ball bounce", &mBallSettings.restitution, 0.0f, 1.0f);
    if (ImGui::Button("Respawn balls"))
    {
        spawnBalls();
    }
}
#endif

#ifdef TEMPLATE_HAS_RELAY
void SandboxGame::initRelay()
{
    mRelay.onLobbyCode = [this](const std::string& code)
    {
        mLobbyCode = code;
        mRelayStatus = "Hosting, waiting for a guest";
    };
    mRelay.onPeerConnected = [this]()
    {
        mRelayStatus = "Peer connected (press R to reset both scenes)";
    };
    mRelay.onMessage = [this](std::span<const std::uint8_t> data)
    {
        if (!data.empty() && data[0] == 1)
        {
            resetScene();
            mPeerResets++;
        }
    };
    mRelay.onError = [this](const std::string& reason)
    {
        mRelayStatus = "Error: " + reason;
    };
    mRelay.onDisconnected = [this]()
    {
        mRelayStatus = "Disconnected";
        mLobbyCode.clear();
    };

    // Proxies drop idle WebSockets, so a waiting host pings the server
    getCallbacks().addTimed(10.0, [this]()
    {
        mRelay.sendKeepalive();
    }, true);
}

void SandboxGame::drawRelayGui()
{
    ImGui::Separator();
    ImGui::Text("Relay (run relay_server first)");
    ImGui::InputText("Server", mRelayUrl, sizeof(mRelayUrl));
    if (ImGui::Button("Host"))
    {
        mRelayStatus = "Connecting";
        mRelay.host(mRelayUrl, "Player");
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(90.0f);
    ImGui::InputText("##code", mJoinCode, sizeof(mJoinCode), ImGuiInputTextFlags_CharsUppercase);
    ImGui::SameLine();
    if (ImGui::Button("Join"))
    {
        mRelayStatus = "Connecting";
        mRelay.join(mRelayUrl, mJoinCode);
    }
    if (!mLobbyCode.empty())
    {
        ImGui::Text("Lobby code: %s", mLobbyCode.c_str());
    }
    ImGui::TextWrapped("%s", mRelayStatus.c_str());
    ImGui::Text("Resets from peer: %d", mPeerResets);
}
#endif
